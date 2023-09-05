require 'rake'
require 'pathname'
require 'fileutils'

verbose(ENV['verbose'] == '1')
DEBUG = ENV['debug'] == '1'
TESTING = ENV['testing'] == '1'

def pop_path(path)
  Pathname(path).each_filename.to_a[1..-1]
end

def obj2src(fn, e)
  File.absolute_path(File.join('src', pop_path(File.dirname(fn)), File.basename(fn).ext(e)))
end

#def obj2src(fn, e)
#  File.join('src', pop_path(File.dirname(fn)), File.basename(fn).ext(e))
#end

def is_windows?
  (/cygwin|mswin|mingw|bccwin|wince|emx/ =~ RUBY_PLATFORM) != nil
end

# Makefile .d file loader to be used with the import file loader.
# this emulates the -include $(DEPFILES) in a Makefile for the generated .d files
class DfileLoader
  include Rake::DSL

  SPACE_MARK = "\0"

  # Load the makefile dependencies in +fn+.
  def load(fn)
    return if ! File.exists?(fn)
    lines = File.read fn
    lines.gsub!(/\\ /, SPACE_MARK)
    lines.gsub!(/#[^\n]*\n/m, "")
    lines.gsub!(/\\\n/, ' ')
    lines.each_line do |line|
      process_line(line)
    end
  end

  private

  # Process one logical line of makefile data.
  def process_line(line)
    file_tasks, args = line.split(':', 2)
    return if args.nil?
    dependents = args.split.map { |d| respace(d) }
    file_tasks.scan(/\S+/) do |file_task|
      file_task = respace(file_task)
      file file_task => dependents
    end
  end

  def respace(str)
    str.tr SPACE_MARK, ' '
  end
end

# Install the handler
Rake.application.add_loader('d', DfileLoader.new)

PROG = 'smoothie'

DEVICE = 'LPC1768'
ARCHITECTURE = 'armv7-m'

MBED_DIR = './mbed/drop'

TOOLSBIN = './gcc-arm-none-eabi/bin/arm-none-eabi-'
CC = "#{TOOLSBIN}gcc"
CCPP = "#{TOOLSBIN}g++"
LD = "#{TOOLSBIN}g++"
OBJCOPY = "#{TOOLSBIN}objcopy"
SIZE = "#{TOOLSBIN}size"

# include a defaults file if present
load 'rakefile.defaults' if File.exists?('rakefile.defaults')
if TESTING
  BUILDTYPE= 'Testing'

elsif DEBUG
  BUILDTYPE= 'Debug'
  ENABLE_DEBUG_MONITOR= '0'
end

# Set build type
BUILDTYPE= ENV['BUILDTYPE'] || 'Checked' unless defined? BUILDTYPE
puts "#{BUILDTYPE} build"

ENABLE_DEBUG_MONITOR = ENV['ENABLE_DEBUG_MONITOR'] || '0' unless defined? ENABLE_DEBUG_MONITOR

# set default baud rate
DEFAULT_SERIAL_BAUD_RATE= ENV['BAUDRATE'] || '115200' unless defined? DEFAULT_SERIAL_BAUD_RATE

# set to true to eliminate all the network code
# unless defined? NONETWORK
#  NONETWORK= false || TESTING
# end
NONETWORK = true

# list of modules to exclude, include directory it is in
# e.g for a CNC machine
#EXCLUDE_MODULES = %w(tools/touchprobe tools/laser tools/temperaturecontrol tools/extruder)

# generate regex of modules to exclude and defines
exclude_defines, excludes = EXCLUDE_MODULES.collect { |e|  [e.tr('/', '_').upcase, e.sub('/', '\/')] }.transpose

# see if network is enabled
if ENV['NONETWORK'] || NONETWORK
  nonetwork= true
  excludes << '\/libs\/Network\/'
  puts "Excluding Network code"
else
  nonetwork= false
end

# see if CNC build
if ENV['CNC'] || CNC
  cnc= true
  excludes << 'panel\/screens\/3dprinter'
  puts "CNC build"
else
  excludes << 'panel\/screens\/cnc'
  cnc= false
end

if TESTING
  # add modules to be tested here
  TESTMODULES= %w(tools/temperatureswitch) unless defined? EXCLUDE_MODULES
  puts "Modules under test: #{TESTMODULES}"
  excludes << %w(Kernel.cpp main.cpp) # we replace these with mock versions in testframework

  frameworkfiles= FileList['src/testframework/*.{c,cpp}', 'src/testframework/easyunit/*.{c,cpp}']
  extrafiles= FileList['src/modules/communication/SerialConsole.cpp', 'src/modules/communication/utils/Gcode.cpp', 'src/modules/robot/Conveyor.cpp', 'src/modules/robot/Block.cpp']
  testmodules= FileList['src/libs/**/*.{c,cpp}'].include(TESTMODULES.collect { |e| "src/modules/#{e}/**/*.{c,cpp}"}).include(TESTMODULES.collect { |e| "src/testframework/unittests/#{e}/*.{c,cpp}"}).exclude(/#{excludes.join('|')}/)
  SRC =  frameworkfiles + extrafiles + testmodules
else
  excludes << %w(testframework)
  SRC = FileList['src/**/*.{c,cpp}'].exclude(/#{excludes.join('|')}/)
  puts "WARNING Excluding modules: #{EXCLUDE_MODULES.join(' ')}" unless exclude_defines.empty?
end

OBJDIR = 'OBJ'
OBJ = SRC.collect { |fn| File.join(OBJDIR, pop_path(File.dirname(fn)), File.basename(fn).ext('o')) } +
	%W(#{OBJDIR}/configdefault.o #{OBJDIR}/mbed_custom.o)

# list of header dependency files generated by compiler
DEPFILES = OBJ.collect { |fn| File.join(File.dirname(fn), File.basename(fn).ext('d')) }

# create destination directories
SRC.each do |s|
  d= File.join(OBJDIR, pop_path(File.dirname(s)))
  FileUtils.mkdir_p(d) unless Dir.exists?(d)
end

INCLUDE_DIRS = [Dir.glob(['./src/**/', './mri/**/'])].flatten
MBED_INCLUDE_DIRS = %W(#{MBED_DIR}/ #{MBED_DIR}/LPC1768/)

INCLUDE = (INCLUDE_DIRS+MBED_INCLUDE_DIRS).collect { |d| "-I#{d}" }.join(" ")

if ENABLE_DEBUG_MONITOR == '1'
  # Can add MRI_UART_BAUD=115200 to next line if GDB fails to connect to MRI.
  # Tends to happen on some Linux distros but not Windows and OS X.
  MRI_UART = 'MRI_UART_0'
  MRI_BREAK_ON_INIT = 1 unless defined? MRI_BREAK_ON_INIT
else
  MRI_UART = 'MRI_UART_0 MRI_UART_SHARE'
  MRI_BREAK_ON_INIT = 0 unless defined? MRI_BREAK_ON_INIT
end

# Configure MRI variables based on BUILD_TYPE build type variable.
case BUILDTYPE.downcase
when 'release'
  OPTIMIZATION = 2
  MRI_ENABLE = 0
  MRI_SEMIHOST_STDIO = 0 unless defined? MRI_SEMIHOST_STDIO
when 'debug'
  OPTIMIZATION = 0
  MRI_ENABLE = 1
  MRI_SEMIHOST_STDIO = 1 unless defined? MRI_SEMIHOST_STDIO
when 'checked'
  OPTIMIZATION = 2
  MRI_ENABLE = 1
  MRI_SEMIHOST_STDIO = 1 unless defined? MRI_SEMIHOST_STDIO
when 'testing'
  OPTIMIZATION = 0
  MRI_ENABLE = 1
  MRI_SEMIHOST_STDIO = 0 unless defined? MRI_SEMIHOST_STDIO
end

MRI_ENABLE = 1  unless defined? MRI_ENABLE # set to 0 to disable MRI
MRI_LIB = MRI_ENABLE == 1 ? './mri/mri.ar' : ''
MBED_LIB = "#{MBED_DIR}/LPC1768/GCC_ARM/libmbed.a"

SYS_LIBS = '-lstdc++_s -lsupc++_s -lm -lgcc -lc_s -lgcc -lc_s -lnosys'
LIBS = [MBED_LIB, SYS_LIBS, MRI_LIB].join(' ')

MRI_DEFINES = %W(-DMRI_ENABLE=#{MRI_ENABLE} -DMRI_INIT_PARAMETERS='"#{MRI_UART}"' -DMRI_BREAK_ON_INIT=#{MRI_BREAK_ON_INIT} -DMRI_SEMIHOST_STDIO=#{MRI_SEMIHOST_STDIO})

defines = %w(-DCHECKSUM_USE_CPP -D__LPC17XX__  -DTARGET_LPC1768 -DWRITE_BUFFER_DISABLE=0 -DSTACK_SIZE=3072 -DCHECKSUM_USE_CPP)
defines += exclude_defines.collect{|d| "-DNO_#{d}"}
defines += MRI_DEFINES
defines << "-DDEFAULT_SERIAL_BAUD_RATE=#{DEFAULT_SERIAL_BAUD_RATE}"
defines << '-DDEBUG' if OPTIMIZATION == 0
defines << '-DNONETWORK' if nonetwork
defines << '-DCNC' if cnc

DEFINES= defines.join(' ')

# Compiler flags used to enable creation of header dependencies.
DEPFLAGS = '-MMD '
CFLAGS = DEPFLAGS + "-Wall -Wextra -Wno-unused-parameter -Wcast-align -Wpointer-arith -Wredundant-decls -Wcast-qual -Wcast-align -O#{OPTIMIZATION} -g3 -mcpu=cortex-m3 -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -fno-delete-null-pointer-checks"
CPPFLAGS = CFLAGS + ' -fno-rtti -std=gnu++11 -fno-exceptions'
CXXFLAGS = CFLAGS + ' -fno-rtti -std=gnu++11 -fexceptions' # used for a .cxx file that needs to be compiled with exceptions

MRI_WRAPS = MRI_ENABLE == 1 ? ',--wrap=_read,--wrap=_write,--wrap=semihost_connected' : ''

# Linker script to be used.  Indicates what code should be placed where in memory.
LSCRIPT = "#{MBED_DIR}/LPC1768/GCC_ARM/LPC1768.ld"
LDFLAGS = "-mcpu=cortex-m3 -mthumb -specs=./build/startfile.spec" +
    " -Wl,-Map=#{OBJDIR}/smoothie.map,--cref,--gc-sections,--wrap=_isatty,--wrap=malloc,--wrap=realloc,--wrap=free" +
    MRI_WRAPS +
    " -T#{LSCRIPT}" +
    " -u _scanf_float -u _printf_float"

HTTPD_FSDATA = './src/libs/Network/uip/webserver/httpd-fsdata2.h'

# tasks

# generate the header dependencies if they exist
import(*DEPFILES)

task :clean do
  FileUtils.rm_rf(OBJDIR)
end

task :realclean => [:clean] do
  sh "cd ./mbed;make clean"
end

desc "Build the built-in web pages"
task :webui => [HTTPD_FSDATA]

task :default => [:build]

task :build => [MBED_LIB, :version, "#{PROG}.bin", :size]

task :version do
  if is_windows?
    VERSION = ' -D__GITVERSIONSTRING__=\"place-holder\"'
  else
    v1= `git symbolic-ref HEAD 2> /dev/null`
    v2= `git log --pretty=format:%h -1`
    VERSION = ' -D__GITVERSIONSTRING__=\"' + "#{v1[11..-1].chomp}-#{v2}".chomp + '\"'
    FileUtils.touch './src/version.cpp' # we want it compiled everytime
  end
end

desc "Upload via DFU"
task :upload do
  sh "dfu-util -R -d 1d50:6015 -D #{OBJDIR}/#{PROG}.bin"
end

task :size do
  sh "#{SIZE} #{OBJDIR}/#{PROG}.elf"
end

# build internal web page
WEB_SOURCE_FILES= FileList['./src/libs/Network/uip/webserver/httpd-fs-src/**/*']
WEBDIR = './src/libs/Network/uip/webserver/httpd-fs'
file HTTPD_FSDATA => WEB_SOURCE_FILES do
  FileUtils.rm_rf WEBDIR
  FileUtils.mkdir WEBDIR
  FileUtils.cp WEB_SOURCE_FILES, WEBDIR
  # TODO minify some files
  # sh 'java -jar yuicompressor-2.4.8.jar file -o file'
  sh 'cd ./src/libs/Network/uip/webserver; perl makefsdata.pl'
  sh 'cd ./src; make'
end

file MBED_LIB do
  puts "Building Mbed Using Make"
  sh "cd ./mbed;make all"
end

file "#{OBJDIR}/mbed_custom.o" => ['./build/mbed_custom.cpp'] do |t|
  puts "Compiling mbed_custom.cpp"
  sh "#{CCPP} #{CPPFLAGS} #{INCLUDE} #{DEFINES} -c -o #{t.name} #{t.prerequisites[0]}"
end

file "#{OBJDIR}/configdefault.o" => 'src/config.default' do |t|
  sh "cd ./src; ../#{OBJCOPY} -I binary -O elf32-littlearm -B arm --readonly-text --rename-section .data=.rodata.configdefault config.default ../#{OBJDIR}/configdefault.o"
end

file "#{PROG}.bin" => ["#{PROG}.elf"] do
  sh "#{OBJCOPY} -O binary #{OBJDIR}/#{PROG}.elf #{OBJDIR}/#{PROG}.bin"
end

file "#{PROG}.elf" => OBJ do |t|
  puts "Linking"
  sh "#{LD} #{LDFLAGS} #{OBJ} #{LIBS}  -o #{OBJDIR}/#{t.name}"
end

#arm-none-eabi-objcopy -R .stack -O ihex ../LPC1768/main.elf ../LPC1768/main.hex
#arm-none-eabi-objdump -d -f -M reg-names-std --demangle ../LPC1768/main.elf >../LPC1768/main.disasm

rule '.o' => lambda{ |objfile| obj2src(objfile, 'cpp') } do |t|
  puts "Compiling #{t.source}"
  sh "#{CCPP} #{CPPFLAGS} #{INCLUDE} #{DEFINES} #{VERSION} -c -o #{t.name} #{t.source}"
end

rule '.o' => lambda{ |objfile| obj2src(objfile, 'cxx') } do |t|
  puts "Compiling #{t.source}"
  sh "#{CCPP} #{CXXFLAGS} #{INCLUDE} #{DEFINES} #{VERSION} -c -o #{t.name} #{t.source}"
end

rule '.o' => lambda{ |objfile| obj2src(objfile, 'c') } do |t|
  puts "Compiling #{t.source}"
  sh "#{CC} #{CFLAGS} #{INCLUDE} #{DEFINES} #{VERSION} -c -o #{t.name} #{t.source}"
end
