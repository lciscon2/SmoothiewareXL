/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ToolManager.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "ToolManagerPublicAccess.h"
#include "Config.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "FileStream.h"

#include <math.h>

ToolManager::ToolManager()
{
    active_tool = 0;
    current_tool_name = CHECKSUM("hotend");
}

void ToolManager::on_module_loaded()
{

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

void ToolManager::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

	//filter out certain lines because they are not tool changes!!
	if(gcode->has_m) {
		if((gcode->m == 104 ) || ( gcode->m == 109 )) { // temperature settings
//			gcode->stream->printf("Skipping temp command\n");
			return;
		}
	}


    if( gcode->has_letter('T') ) {

        int new_tool = gcode->get_value('T');
		gcode->stream->printf("Toolmanager new tool: %d\n", new_tool);

        if(new_tool >= (int)this->tools.size() || new_tool < 0) {
            // invalid tool
            char buf[32]; // should be big enough for any status
            int n = snprintf(buf, sizeof(buf), "T%d invalid tool ", new_tool);
            gcode->txt_after_ok.append(buf, n);

        } else {
            if(new_tool != this->active_tool) {
                // We must wait for an empty queue before we can disable the current extruder
                THEKERNEL->conveyor->wait_for_idle();
                this->tools[active_tool]->deselect();
                this->active_tool = new_tool;
                this->current_tool_name = this->tools[active_tool]->get_name();
                this->tools[active_tool]->select();

                //send new_tool_offsets to robot
                const float *new_tool_offset = tools[new_tool]->get_offset();
				gcode->stream->printf("Tool offset: %f %f %f\n", new_tool_offset[0],new_tool_offset[1],new_tool_offset[2]);
                THEROBOT->set_tool_offset(new_tool_offset);
            }
        }
	} else if(gcode->has_m) {
	// M code processing here

	switch (gcode->m) {
		case 676: //get current tool #
			gcode->stream->printf("%d\n",active_tool);
			break;

		case 675: //set tool offsets
			if(gcode->subcode == 1) {
				const float *old_tool_offset = tools[1]->get_offset();
				float newoff[3] = {old_tool_offset[0],old_tool_offset[1],0.0};
				if (gcode->has_letter('X')) newoff[0] = gcode->get_value('X');
				if (gcode->has_letter('Y')) newoff[1] = gcode->get_value('Y');
				tools[1]->set_offset(0, newoff[0]);
				tools[1]->set_offset(1, newoff[1]);
				tools[1]->set_offset(2, newoff[2]);
//				tools[1]->set_offset(newoff);
			}
			break;

		case 500: // save settings
		case 503: // print settings
			{
			//BUGBUG HACKHACK FIXFIX only shows tool offset for T1 right now
			const float *old_tool_offset2 = tools[1]->get_offset();
			gcode->stream->printf(";Tool offsets:\nM675.1 X%1.4f Y%1.4f\n",
				old_tool_offset2[0],old_tool_offset2[1]);
			}
			break;

		case 553:
			{
			const float *old_tool_offset4 = tools[0]->get_offset();
			gcode->stream->printf(";T0 Tool offsets: X%1.4f Y%1.4f Z%1.4f\n",
			old_tool_offset4[0],old_tool_offset4[1],old_tool_offset4[2]);
			const float *old_tool_offset3 = tools[1]->get_offset();
			gcode->stream->printf(";T1 Tool offsets: X%1.4f Y%1.4f Z%1.4f\n",
				old_tool_offset3[0],old_tool_offset3[1],old_tool_offset3[2]);
			}
			break;
		}
	}


    if(gcode->has_g && gcode->g == 10 && gcode->has_letter('L') && gcode->get_int('L') == 1 && gcode->has_letter('P')) {
        // Handle G10 L1 Pn Xnnn Ynnn Znnn
        size_t n = gcode->get_uint('P')-1;
        if(n < this->tools.size()) {
            // Set the tool offset for this tool
            if(gcode->has_letter('X')) tools[n]->set_offset(0, gcode->get_value('X'));
            if(gcode->has_letter('Y')) tools[n]->set_offset(1, gcode->get_value('Y'));
            if(gcode->has_letter('Z')) tools[n]->set_offset(2, gcode->get_value('Z'));
            if((int)n == this->active_tool) {
                // send updated tool_offsets to robot
                const float *new_tool_offset = tools[n]->get_offset();
                THEROBOT->set_tool_offset(new_tool_offset);
            }

        }else{
            gcode->stream->printf("Error:invalid tool: %u\n", n);
        }
    }
}

void ToolManager::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(tool_manager_checksum)) return;

    if(pdr->second_element_is(is_active_tool_checksum)) {

        // check that we control the given tool
        bool managed = false;
        for(auto t : tools) {
            uint16_t n = t->get_name();
            if(pdr->third_element_is(n)) {
                managed = true;
                break;
            }
        }

        // we are not managing this tool so do not answer
        if(!managed) return;

        pdr->set_data_ptr(&this->current_tool_name);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_active_tool_checksum)) {
        pdr->set_data_ptr(&this->active_tool);
        pdr->set_taken();
    }
}

void ToolManager::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(tool_manager_checksum)) return;

	if(pdr->second_element_is(set_offset_checksum)) {
       float *info = static_cast<float *>(pdr->get_data_ptr());
       int toolnum = info[0];
       int axis = info[1];
       float newoffset = info[2];

       float *offset = this->tools[toolnum]->get_offset();
       offset[axis] = newoffset;
     }

    // ok this is targeted at us, so change tools
    //uint16_t tool_name= *static_cast<float*>(pdr->get_data_ptr());
    // TODO: fire a tool change gcode
    //pdr->set_taken();
}

// Add a tool to the tool list
void ToolManager::add_tool(Tool* tool_to_add)
{
    if(this->tools.size() == 0) {
        tool_to_add->select();
        this->current_tool_name = tool_to_add->get_name();
        //send new_tool_offsets to robot
        const float *new_tool_offset = tool_to_add->get_offset();
        THEROBOT->set_tool_offset(new_tool_offset);
    } else {
        tool_to_add->deselect();
    }
    this->tools.push_back( tool_to_add );
}
