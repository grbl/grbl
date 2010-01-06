require 'rubygems'
require 'optparse'
require 'serialport'


options_parser = OptionParser.new do |opts|
  opts.banner = "Usage: stream [options] gcode-file"
  opts.on('-v', '--verbose', 'Output more information') do
    $verbose = true
  end   
  
  opts.on('-h', '--help', 'Display this screen') do
    puts opts
    exit
  end
end
options_parser.parse!
if ARGV.empty?
  puts options_parser
  exit
end

SerialPort.open('/dev/tty.usbserial-A4001o6L', 9600) do |sp|
  ARGV.each do |file|
    puts "Processing file #{file}"
    prebuffer = 5
    File.readlines(file).each do |line|
      sp.write("#{line.strip}\r\n");
      puts line.strip
      if prebuffer == 0
        result = sp.gets 
        puts result unless result.strip == 'ok'
      else
        prebuffer -= 1
      end
    end
  end
end