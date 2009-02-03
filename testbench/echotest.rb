require 'pp'
require 'rubygems'
require 'serialport'
require 'png'

# Requires the gems serialport and png
# 
# $ gem install ruby-serialport
# $ gem install png


@sp = SerialPort.new '/dev/tty.usbserial-A4001o6L', 19200

@canvas = PNG::Canvas.new(500,500)
@x = 0
@y = 0
@ready_for_command = true
@last_byte = Time.now

@input = ''

@step_count = 0

Thread.new do
  step_mode = false
  @sp.each_byte do |byte| 
    #puts byte.chr
       
    @last_byte = Time.now
    @input << byte.chr
    if byte == 126 # ('~')
      step_mode = true          
      @input << '[step mode on]'
    else
      if step_mode 
        @step_count += 1
        if byte == 10
          @input << '[step mode off]'
          step_mode = false
        else
          step = byte-33
          puts @step_count
          if (step & 1) != 0
            if (step & 8) != 0
              puts "x-1"
              @x+=1
            else
              puts "x+1"
              @x-=1
            end
          end
          if (step & 2) != 0
            if (step & 16) != 0
              puts "y-1"
              @y+=1
            else
              puts "y+1"
              @y-=1
            end
          end
        end
      else
        if byte == 62 # '>'
          @ready_for_command = true
        end
      end
    end
    @canvas[250-@x,250-@y] = PNG::Color::Black    
  end
end

def send_command(command)
  while !@ready_for_command do
    sleep(1)
    puts "Processing ..."
    if (Time.now-@last_byte > 5) 
      return
    end
  end
  sleep(0.5)
  puts "Sent: #{command}"
  @input << "[sent #{command}]"
  @sp.write("#{command}\r")
  @ready_for_command = false
end


blocks = <<-END.split("\n").map{|s|s.strip}
N10 G00 Z100.000  G53
N15 G00 X38.105 Y71.468  G53
N20 Z2.0 F200  G53
N25 G01 Z-0.2 G53
N30 X37.177 Y74.406 G53
N35 X37.779 Y74.698 G53
N40 X41.077 Y76.347 G53
N45 X42.177 Y73.719 G53
N50 X40.304 Y72.173 G53
N55 X39.548 Y73.873 G53
N60 X38.638 Y73.152 G53
N65 X39.153 Y71.915 G53
N70 X38.105 Y71.468 G53
N75 G00 Z2.0 G53
N80 X37.266 Y75.042  G53
N85 G01 Z-0.2 G53
N90 X37.862 Y79.198 G53
N95 G00 Z2.0 G53
N100 X37.684 Y77.960  G53
N105 G01 Z-0.2 G53
N110 X42.718 Y77.239 G53
N115 G00 Z2.0 G53
N120 X42.895 Y78.476  G53
N125 G01 Z-0.2 G53
N130 X42.300 Y74.320 G53
N135 G00 Z2.0 G53
N140 X43.975 Y77.690  G53
N145 G01 Z-0.2 G53
N150 X43.425 Y77.769 G53
N155 G00 Z2.0 G53
N160 X44.604 Y78.161  G53
N165 G01 Z-0.2 G53
N170 G02 X43.975 Y77.690 I-0.550 J0.079 G53
N175 G00 Z2.0 G53
N180 X44.643 Y78.436  G53
N185 G01 Z-0.2 G53
N190 X44.604 Y78.161 G53
N195 G00 Z2.0 G53
N200 X44.172 Y79.065  G53
N205 G01 Z-0.2 G53
N210 G02 X44.643 Y78.436 I-0.079 J-0.550 G53
N215 G00 Z2.0 G53
N220 X43.622 Y79.144  G53
N225 G01 Z-0.2 G53
N230 X44.172 Y79.065 G53
N235 G00 Z2.0 G53
N240 X43.780 Y80.243  G53
N245 G01 Z-0.2 G53
N250 X43.622 Y79.144 G53
N255 G00 Z2.0 G53
N260 X44.880 Y80.086  G53
N265 G01 Z-0.2 G53
N270 X43.780 Y80.243 G53
N275 G00 Z2.0 G53
N280 X46.488 Y79.154  G53
N285 G01 Z-0.2 G53
N290 G02 X46.808 Y79.669 I1.265 J-0.427 G53
N295 G02 X47.312 Y79.597 I0.229 J-0.198 G53
N300 G02 X47.474 Y79.012 I-1.172 J-0.639 G53
N305 G02 X47.452 Y78.454 I-2.606 J-0.179 G53
N310 G02 X47.316 Y77.912 I-2.587 J0.362 G53
N315 G02 X46.997 Y77.397 I-1.265 J0.427 G53
N320 G02 X46.493 Y77.469 I-0.229 J0.198 G53
N325 G02 X46.331 Y78.054 I1.172 J0.639 G53
N330 G02 X46.352 Y78.612 I2.606 J0.179 G53
N335 G02 X46.489 Y79.154 I2.587 J-0.362 G53
N340 G00 Z2.0 G53
N345 X45.370 Y77.630  G53
N350 G01 Z-0.2 G53
N355 X45.350 Y77.493 G53
N360 G00 Z2.0 G53
N365 X48.179 Y77.649  G53
N370 G01 Z-0.2 G53
N375 X48.218 Y77.924 G53
N380 G02 X49.318 Y77.766 I0.550 J-0.079 G53
N385 G01 X49.278 Y77.491 G53
N390 G02 X48.179 Y77.649 I-0.550 J0.079 G53
N395 G00 Z2.0 G53
N400 X48.435 Y78.945  G53
N405 G01 Z-0.2 G53
N410 G03 X49.397 Y78.807 I0.481 J-0.069 G53
N415 G01 X49.417 Y78.944 G53
N420 G03 X48.454 Y79.082 I-0.481 J0.069 G53
N425 G01 X48.435 Y78.945 G53
N430 G00 Z2.0 G53
N435 X50.379 Y79.297  G53
N440 G01 Z-0.2 G53
N445 X51.479 Y79.140 G53
N450 G00 Z2.0 G53
N455 X50.222 Y78.198  G53
N460 G01 Z-0.2 G53
N465 X50.379 Y79.297 G53
N470 G00 Z2.0 G53
N475 X50.772 Y78.119  G53
N480 G01 Z-0.2 G53
N485 X50.222 Y78.198 G53
N490 G00 Z2.0 G53
N495 X51.243 Y77.490  G53
N500 G01 Z-0.2 G53
N505 G03 X50.772 Y78.119 I-0.550 J0.079 G53
N510 G00 Z2.0 G53
N515 X51.203 Y77.215  G53
N520 G01 Z-0.2 G53
N525 X51.243 Y77.490 G53
N530 G00 Z2.0 G53
N535 X50.574 Y76.744  G53
N540 G01 Z-0.2 G53
N545 G03 X51.203 Y77.215 I0.079 J0.550 G53
N550 G00 Z2.0 G53
N555 X50.025 Y76.823  G53
N560 G01 Z-0.2 G53
N565 X50.574 Y76.744 G53
N570 G00 Z2.0 G53
N575 X53.049 Y76.389  G53
N580 G01 Z-0.2 G53
N585 X51.949 Y76.547 G53
N590 X53.274 Y78.222 G53
N595 G03 X53.327 Y78.454 I-0.218 J0.172 G53
N600 G03 X52.262 Y78.607 I-0.552 J-0.061 G53
N605 G00 Z2.0 G53
N610 M30 G53
END

blocks.each do |block|
  send_command(block)
  if (Time.now-@last_byte > 5) 
    puts "Bailing, cause somethin' went wrong"
    break
  end
end

sleep(3);

PNG.new(@canvas).save("gcodetest.png")

puts @input
`open gcodetest.png`
