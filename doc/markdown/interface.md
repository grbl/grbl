# Grbl Interface Basics

The interface for Grbl is fairly simple and straightforward. With Grbl v1.1, steps have been taken to try to make it even easier for new users to get started, and for GUI developers to write their own custom interfaces to Grbl.

Grbl communicates through the serial interface on the Arduino. You just need to connect your Arduino to your computer with a USB cable. Use any standard serial terminal program to connect to Grbl, such as: the Arduino IDE serial monitor, Coolterm, puTTY, etc. Or use one of the many great Grbl GUIs out there in the Internet wild.

The primary way to talk to Grbl is performed by sending it a string of characters, followed by a carriage return. Grbl will then process the string, set it up for execution, and then reply back with a **response message**, also terminated by a return, to tell you how it went. These command strings include sending Grbl: a G-code block to execute, commands to configure Grbl's system settings, to view how Grbl is doing, etc. 

To stream a g-code program to Grbl, the basic interface is to send Grbl a line of g-code, then wait for the proper **response message** starting with an `ok` or `error`. This signals Grbl has completed the parsing and executing the command. At times, Grbl may not respond immediately. This happens when Grbl is busy doing something else or waiting to place a commanded motion into the look-ahead planner buffer. Other times, usually at the start of a program, Grbl may quickly respond to several lines, but nothing happens. This occurs when Grbl places a series of commanded motions directly in the planner queue and will try to fill it up completely before starting.

Along with **response messages**, Grbl has **push messages** to provide more feedback on what Grbl is doing and are also strings terminated by a return. These messages may be "pushed" from Grbl to the user in response to a query or to let the user know something important just happened. These can come at any time, but usually from something like a settings print out when asked to. **Push messages** are easily identified because they don't start with an `ok` or `error` like **response messages** do. They are typically placed in `[]` brackets, `<>` chevrons, start with a `$`, or a specific string of text. These are all defined and described later in this document.

Finally, Grbl has **real-time commands** that are invoked by a set of special characters that may be sent at any time and are not part of the basic streaming send-response interface. These cause Grbl to immediately execute the command and typically don't generate a response. These include pausing the current motion, speed up/down everything, toggle the spindle during a job, reset Grbl, or query Grbl for a real-time status report. See the `Commands` document to see what they are and how they work.

-------

# Writing an Interface for Grbl

The general interface for Grbl has been described above, but what's missing is how to run an entire G-code program on Grbl, when it doesn't seem to have an upload feature. This is where this section fits in. Early on, users fiercely requested for flash drive, external RAM, LCD support, joysticks, or network support so they can upload a g-code program and run it directly on Grbl. The general answer to that is, good ideas, but Grbl doesn't need them. Grbl already has nearly all of the tools and features to reliably communicate with a graphical user interface (GUI) or a seperate host interface that provides all those extra bells and whistles. Grbl's base philosophy is to minimize what Grbl should be doing, because, in the end, Grbl needs to be concentrating on producing clean, reliable motion. That's it.


## Streaming a G-Code Program to Grbl

Here we will describe two different streaming methods for Grbl GUIs. One of the main problems with streaming to Grbl is the USB port itself. Arduinos and most all micro controllers use a USB-to-serial converter chip that, at times, behaves strangely and not typically how you'd expect, like USB packet buffering and delays that can wreak havoc to a streaming protocol. Another problem is how to deal with some of the latency and oddities of the PCs themselves, because none of them are truly real-time and always create micro-delays when executing other tasks. Regardless, we've come up with ways to ensure the G-code stream is reliable and simple. 

The following streaming protocols require tracking the **response messages** to determine when to send the next g-code line. All **push messages** are not counted toward the streaming protocol and should be handled separately. All real-time command characters can be sent at any time and are never placed in Grbl's RX serial buffer. They are intercepted as they come in and simply sets  flags for Grbl to execute them.

#### Streaming Protocol: Simple Send-Response _[Recommended]_
The send-response streaming protocol is the most fool-proof and simplest method to stream a G-code program to Grbl. The host PC interface simply sends a line of G-code to Grbl and waits for an `ok` or `error:` **response message** before sending the next line of G-code. So, no matter if Grbl needs to wait for room in the look-ahead planner buffer to finish parsing and executing the last line of G-code or if the the host computer is busy doing something, this guarantees both to the host PC and Grbl, the programmed G-code has been sent and received properly. An example of this protocol is published in our `simple_stream.py` script in our repository.

However, it's also the slowest of three outlined streaming protocols. Grbl essentially has two buffers between the execution of steps and the host PC interface. One of them is the serial receive buffer. This briefly stores up to 127 characters of data received from the host PC until Grbl has time to fetch and parse the line of G-code. The other buffer is the look-ahead planner buffer. This buffer stores up to 16 line motions that are acceleration-planned and optimized for step execution. Since the send-response protocol receives a line of G-code while the host PC waits for a response, Grbl's serial receive buffer is usually empty and under-utilized. If Grbl is actively running and executing steps, Grbl will immediately begin to execute and empty the look-ahead planner buffer, while it sends the response to the host PC, waits for the next line from the host PC, upon receiving it, parse and plan it, and add it to the end of the look-ahead buffer.

Although this communication lag may take only a fraction of a second, there is a cumulative effect, because there is a lag with every G-code block sent to Grbl. In certain scenarios, like a G-code program containing lots of sequential, very short, line segments with high feed rates, the cumulative lag can be large enough to empty and starve the look-ahead planner buffer within this time. This could lead to start-stop motion when the streaming can't keep up with G-code program execution. Also, since Grbl can only plan and optimize what's in the look-ahead planner buffer, the performance through these types of motions will never be full-speed, because look-ahead buffer will always be partially full when using this streaming method. If your expected application doesn't contain a lot of these short line segments with high feed rates, this streaming protocol should be more than adequate for a vast majority of applications, is very robust, and is a quick way to get started.

#### Streaming Protocol: Character-Counting _[**Recommended with Reservation**]_

To get the best of both worlds, the simplicity and reliability of the send-response method and assurance of maximum performance with software flow control, we came up with a simple character-counting protocol for streaming a G-code program to Grbl. It works like the send-response method, where the host PC sends a line of G-code for Grbl to execute and waits for a `response message`, but, rather than needing special XON/XOFF characters for flow control, this protocol simply uses Grbl's responses as a way to reliably track how much room there is in Grbl's serial receive buffer. An example of this protocol is outlined in the `stream.py` streaming script in our repo. This protocol is particular useful for very fast machines like laser cutters. 

The main difference between this protocol and the others is the host PC needs to maintain a standing count of how many characters it has sent to Grbl and then subtract the number of characters corresponding to the line executed with each Grbl response. Suppose there is a short G-code program that has 5 lines with 25, 40, 31, 58, and 20 characters (counting the line feed and carriage return characters too). We know Grbl has a 128 character serial receive buffer, and the host PC can send up to 128 characters without overflowing the buffer. If we let the host PC send as many complete lines as we can without over flowing Grbl's serial receive buffer, the first three lines of 25, 40, and 31 characters can be sent for a total of 96 characters. When Grbl sends a **response message**, we know the first line has been processed and is no longer in the serial read buffer. As it stands, the serial read buffer now has the 40 and 31 character lines in it for a total of 71 characters. The host PC needs to then determine if it's safe to send the next line without overflowing the buffer. With the next line at 58 characters and the serial buffer at 71 for a total of 129 characters, the host PC will need to wait until more room has cleared from the serial buffer. When the next Grbl **response message** comes in, the second line has been processed and only the third 31 character line remains in the serial buffer. At this point, it's safe to send the remaining last two 58 and 20 character lines of the g-code program for a total of 110.

While seemingly complicated, this character-counting streaming protocol is extremely effective in practice. It always ensures Grbl's serial read buffer is filled, while never overflowing it. It maximizes Grbl's performance by keeping the look-ahead planner buffer full by better utilizing the bi-directional data flow of the serial port, and it's fairly simple to implement as our `stream.py` script illustrates. We have stress-tested this character-counting protocol to extremes and it has not yet failed. Seemingly, only the speed of the serial connection is the limit.

_RESERVATION:_

- _If a g-code line is parsed and generates an error **response message**, a GUI should stop the stream immediately. However, since the character-counting method stuffs Grbl's RX buffer, Grbl will continue reading from the RX buffer and parse and execute the commands inside it. A GUI won't be able to control this. The interim solution is to check all of the g-code via the $C check mode, so all errors are vetted prior to streaming. This will get resolved in later versions of Grbl._


## Interacting with Grbl's Systems

Along with streaming a G-code program, there a few more things to consider when writing a GUI for Grbl, such as how to use status reporting, real-time control commands, dealing with EEPROM, and general message handling.

#### Status Reporting
When a `?` character is sent to Grbl (no additional line feed or carriage return character required), it will immediately respond with something like `<Idle|MPos:0.000,0.000,0.000|FS:0.0,0>` to report its state and current position. The `?` is always picked-off and removed from the serial receive buffer whenever Grbl detects one. So, these can be sent at any time. Also, to make it a little easier for GUIs to pick up on status reports, they are always encased by `<>` chevrons.

Developers can use this data to provide an on-screen position digital-read-out (DRO) for the user and/or to show the user a 3D position in a virtual workspace. We recommend querying Grbl for a `?` real-time status report at no more than 5Hz. 10Hz may be possible, but at some point, there are diminishing returns and you are taxing Grbl's CPU more by asking it to generate and send a lot of position data.

Grbl's status report is fairly simply in organization. It always starts with a word describing the machine state like `IDLE` (descriptions of these are available elsewhere in the Wiki). The following data values are usually in the order listed below and separated by `|` pipe characters, but may not be in the exact order or printed at all. For a complete description of status report formatting, read the _Real-time Status Reports_ section below.

#### Real-Time Control Commands
The real-time control commands, `~` cycle start/resume, `!` feed hold,  `^X` soft-reset, and all of the override commands, all immediately signal Grbl to change its running state. Just like `?` status reports, these control characters are picked-off and removed from the serial buffer when they are detected and do not require an additional line-feed or carriage-return character to operate.

One important note are the override command characters. These are defined in the extended-ASCII character space and are generally not type-able on a keyboard. A GUI must be able to send these 8-bit values to support overrides. 

#### EEPROM Issues
EEPROM access on the Arduino AVR CPUs turns off all of the interrupts while the CPU _writes_ to EEPROM. This poses a problem for certain features in Grbl, particularly if a user is streaming and running a g-code program, since it can pause the main step generator interrupt from executing on time. Most of the EEPROM access is restricted by Grbl when it's in certain states, but there are some things that developers need to know.

* Settings should not be streamed with the character-counting streaming protocols. Only the simple send-response protocol works. This is because during the EEPROM write, the AVR CPU also shuts-down the serial RX interrupt, which means data can get corrupted or lost. This is safe with the send-response protocol, because it's not sending data after commanding Grbl to save data.

For reference:
* Grbl's EEPROM write commands: `G10 L2`, `G10 L20`, `G28.1`, `G30.1`, `$x=`, `$I=`, `$Nx=`, `$RST=`
* Grbl's EEPROM read commands: `G54-G59`, `G28`, `G30`, `$$`, `$I`, `$N`, `$#`

#### G-code Error Handling

Grbl's g-code parser is fully standards-compilant with complete error-checking. When a G-code parser detects an error in a G-code block/line, the parser will dump everything in the block from memory and report an `error:` back to the user or GUI. This dump is absolutely the right thing to do, because a g-code line with an error can be interpreted in multiple ways. However, this dump can be problematic, because the bad G-code block may have contained some valuable positioning commands or feed rate settings that the following g-code depends on.

It's highly recommended to do what all professional CNC controllers do when they detect an error in the G-code program, _**halt**_. Don't do anything further until the user has modified the G-code and fixed the error in their program. Otherwise, bad things could happen.

As a service to GUIs, Grbl has a "check G-code" mode, enabled by the `$C` system command. GUIs can stream a G-code program to Grbl, where it will parse it, error-check it, and report `ok`'s and `errors:`'s without powering on anything or moving. So GUIs can pre-check the programs before streaming them for real. To disable the "check G-code" mode, send another `$C` system command and Grbl will automatically soft-reset to flush and re-initialize the G-code parser and the rest of the system. This perhaps should be run in the background when a user first loads a program, before a user sets up his machine. This flushing and re-initialization clears `G92`'s by G-code standard, which some users still incorrectly use to set their part zero.

#### Jogging

As of Grbl v1.1, a new jogging feature is available that accepts incremental, absolute, or absolute override motions, along with a jog cancel real-time command that will automatically feed hold and purge the planner buffer. The most important aspect of the new jogging motion is that it is completely independent from the g-code parser, so GUIs no longer have to ensure the g-code modal states are set back correctly after jogging is complete. See the jogging document for more details on how it works and how you can use it with an analog joystick or rotary dial.

#### Synchronization

For situations when a GUI needs to run a special set of commands for tool changes, auto-leveling, etc, there often needs to be a way to know when Grbl has completed a task and the planner buffer is empty. The absolute simplest way to do this is to insert a `G4 P0.01` dwell command, where P is in seconds and must be greater than 0.0. This acts as a quick force-synchronization and ensures the planner buffer is completely empty before the GUI sends the next task to execute.

-----
# Message Summary

In v1.1, Grbl's interface protocol has been tweaked in the attempt to make GUI development cleaner, clearer, and hopefully easier. All messages are designed to be deterministic without needing to know the context of the message. Each can be inferred to a much greater degree than before just by the message type, which are all listed below.

- **Response Messages:** Normal send command and execution response acknowledgement. Used for streaming.

	- `ok` : Indicates the command line received was parsed and executed (or set to be executed).
	- `error:x` : Indicated the command line received contained an error, with an error code `x`, and was purged. See error code section below for definitions.

- **Push Messages:**
	
	- `< >` : Enclosed chevrons contains status report data.
	- `Grbl X.Xx ['$' for help]` : Welcome message indicates initialization.
	- `ALARM:x` : Indicates an alarm has been thrown. Grbl is now in an alarm state.	
	- `$x=val` and `$Nx=line` indicate a settings printout from a `$` and `$N` user query, respectively.
	- `[MSG:]` : Indicates a non-queried feedback message.
	- `[GC:]` : Indicates a queried `$G` g-code state message.
	- `[HLP:]` : Indicates the help message.
	- `[G54:]`, `[G55:]`, `[G56:]`, `[G57:]`, `[G58:]`, `[G59:]`, `[G28:]`, `[G30:]`, `[G92:]`, `[TLO:]`, and `[PRB:]` messages indicate the parameter data printout from a `$#` user query.
	- `[VER:]` : Indicates build info and string from a `$I` user query.
	- `[echo:]` : Indicates an automated line echo from a pre-parsed string prior to g-code parsing. Enabled by config.h option.
	- `>G54G20:ok` : The open chevron indicates startup line execution. The `:ok` suffix shows it executed correctly without adding an unmatched `ok` response on a new line.

In addition, all `$x=val` settings, `error:`, and `ALARM:` messages no longer contain human-readable strings, but rather codes that are defined in other documents. The `$` help message is also reduced to just showing the available commands. Doing this saves incredible amounts of flash space. Otherwise, the new overrides features would not have fit.

Other minor changes and bug fixes that may effect GUI parsing include:

- Floating point values printed with zero precision do not show a decimal, or look like an integer. This includes spindle speed RPM and feed rate in mm mode.
- `$G` reports fixed a long time bug with program modal state. It always showed `M0` program pause when running. Now during a normal program run, no program modal state is given until an `M0`, `M2`, or `M30` is active and then the appropriate state will be shown.

On a final note, this interface tweak came about out of necessity, as more data is being sent back from Grbl and it is capable of doing many more things. It's not intended to be altered again in the near future, if at all. This is likely the only and last major change to this. If you have any comments or suggestions before Grbl v1.1 goes to master, please do immediately so we can all vet the new alteration before its installed.





---------

# Grbl Response Messages

Every G-code block sent to Grbl and Grbl `$` system command that is terminated with a return will be parsed and processed by Grbl. Grbl will then respond either if it recognized the command with an `ok` line or if there was a problem with an `error` line.

* **`ok`**: All is good! Everything in the last line was understood by Grbl and was successfully processed and executed.

  - If an empty line with only a return is sent to Grbl, it considers it a valid line and will return an `ok` too, except it didn't do anything.


* **`error:X`**: Something went wrong! Grbl did not recognize the command and did not execute anything inside that message. The `X` is given as a numeric error code to tell you exactly what happened. The table below decribes every one of them.

	| ID | Error Code Description |
|:-------------:|----|
| **`1`** | G-code words consist of a letter and a value. Letter was not found. |
| **`2`** | Numeric value format is not valid or missing an expected value. |
| **`3`** | Grbl '$' system command was not recognized or supported. |
| **`4`** | Negative value received for an expected positive value. |
| **`5`** | Homing cycle is not enabled via settings. |
| **`6`** | Minimum step pulse time must be greater than 3usec |
| **`7`** | EEPROM read failed. Reset and restored to default values. |
| **`8`** | Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job. |
| **`9`** | G-code locked out during alarm or jog state |
| **`10`** | Soft limits cannot be enabled without homing also enabled. |
| **`11`** | Max characters per line exceeded. Line was not processed and executed. |
| **`12`** | (Compile Option) Grbl '$' setting value exceeds the maximum step rate supported. |
| **`13`** | Safety door detected as opened and door state initiated. |
| **`14`** | (Grbl-Mega Only) Build info or startup line exceeded EEPROM line length limit. |
| **`15`** | Jog target exceeds machine travel. Command ignored. |
| **`16`** | Jog command with no '=' or contains prohibited g-code. |
| **`17`** | Laser mode disabled. Requires PWM output. |
| **`20`** | Unsupported or invalid g-code command found in block. |
| **`21`** | More than one g-code command from same modal group found in block.|
| **`22`** | Feed rate has not yet been set or is undefined. |
| **`23`** | G-code command in block requires an integer value. |
| **`24`** | Two G-code commands that both require the use of the `XYZ` axis words were detected in the block.|
| **`25`** | A G-code word was repeated in the block.|
| **`26`** | A G-code command implicitly or explicitly requires `XYZ` axis words in the block, but none were detected.|
| **`27`**| `N` line number value is not within the valid range of `1` - `9,999,999`. |
| **`28`** | A G-code command was sent, but is missing some required `P` or `L` value words in the line. |
| **`29`** | Grbl supports six work coordinate systems `G54-G59`. `G59.1`, `G59.2`, and `G59.3` are not supported.|
| **`30`**| The `G53` G-code command requires either a `G0` seek or `G1` feed motion mode to be active. A different motion was active.|
| **`31`** | There are unused axis words in the block and `G80` motion mode cancel is active.|
| **`32`** | A `G2` or `G3` arc was commanded but there are no `XYZ` axis words in the selected plane to trace the arc.|
| **`33`** | The motion command has an invalid target. `G2`, `G3`, and `G38.2` generates this error, if the arc is impossible to generate or if the probe target is the current position.|
| **`34`** | A `G2` or `G3` arc, traced with the radius definition, had a mathematical error when computing the arc geometry. Try either breaking up the arc into semi-circles or quadrants, or redefine them with the arc offset definition.|
| **`35`** | A `G2` or `G3` arc, traced with the offset definition, is missing the `IJK` offset word in the selected plane to trace the arc.|
| **`36`** | There are unused, leftover G-code words that aren't used by any command in the block.|
| **`37`** | The `G43.1` dynamic tool length offset command cannot apply an offset to an axis other than its configured axis. The Grbl default axis is the Z-axis.|
| **`38`** | Tool number greater than max supported value.|


----------------------

# Grbl Push Messages

Along with the response message to indicate successfully executing a line command sent to Grbl, Grbl provides additional push messages for important feedback of its current state or if something went horribly wrong. These messages are "pushed" from Grbl and may appear at anytime. They are usually in response to a user query or some system event that Grbl needs to tell you about immediately. These push messages are organized into six general classes:

- **_Welcome message_** - A unique message to indicate Grbl has initialized.

- **_ALARM messages_** - Means an emergency mode has been enacted and shut down normal use.

- **_'$' settings messages_** - Contains the type and data value for a Grbl setting.

- **_Feedback messages_** - Contains general feedback and can provide useful data.

- **_Startup line execution_** - Indicates a startup line as executed with the line itself and how it went.

- **_Real-time status reports_** - Contains current run data like state, position, and speed.


------

#### Welcome Message

**`Grbl X.Xx ['$' for help]`**

The start up message always prints upon startup and after a reset. Whenever you see this message, this also means that Grbl has completed re-initializing all its systems, so everything starts out the same every time you use Grbl.

* `X.Xx` indicates the major version number, followed by a minor version letter. The major version number indicates the general release, while the letter simply indicates a feature update or addition from the preceding minor version letter.
* Bug fix revisions are tracked by the build info version number, printed when an `$I` command is sent. These revisions don't update the version number and are given by date revised in year, month, and day, like so `20161014`.

-----

#### Alarm Message

Alarm is an emergency state. Something has gone terribly wrong when these occur. Typically, they are caused by limit error when the machine has moved or wants to move outside the machine travel and crash into the ends. They also report problems if Grbl is lost and can't guarantee positioning or a probe command has failed. Once in alarm-mode, Grbl will lock out all g-code functionality and accept only a small set of commands. It may even stop everything and force you to acknowledge the problem until you issue Grbl a reset. While in alarm-mode, the user can override the alarm manually with a specific command, which then re-enables g-code so you can move the machine again. This ensures the user knows about the problem and has taken steps to fix or account for it.

Similar to error messages, all alarm messages are sent as  **`ALARM:X`**, where `X` is an alarm code to tell you exacly what caused the alarm. The table below describes the meaning of each alarm code.

| ID | Alarm Code Description |
|:-------------:|----|
| **`1`** | Hard limit triggered. Machine position is likely lost due to sudden and immediate halt. Re-homing is highly recommended. |
| **`2`** | G-code motion target exceeds machine travel. Machine position safely retained. Alarm may be unlocked. |
| **`3`** | Reset while in motion. Grbl cannot guarantee position. Lost steps are likely. Re-homing is highly recommended. |
| **`4`** | Probe fail. The probe is not in the expected initial state before starting probe cycle, where G38.2 and G38.3 is not triggered and G38.4 and G38.5 is triggered. |
| **`5`** | Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4. |
| **`6`** | Homing fail. Reset during active homing cycle. |
| **`7`** | Homing fail. Safety door was opened during active homing cycle. |
| **`8`** | Homing fail. Cycle failed to clear limit switch when pulling off. Try increasing pull-off setting or check wiring. |
| **`9`** | Homing fail. Could not find limit switch within search distance. Defined as `1.5 * max_travel` on search and `5 * pulloff` on locate phases. |

-------

#### Grbl `$` Settings Message

When a push message starts with a `$`, this indicates Grbl is sending a setting and its configured value. There are only two types of settings messages: a single setting and value `$x=val` and a startup string setting `$Nx=line`. See [Configuring Grbl v1.x] document if you'd like to learn how to write these values for your machine.

- `$x=val` will only appear when the user queries to print all of Grbl's settings via the `$$` print settings command. It does so sequentially and completes with an `ok`.

  - In prior versions of Grbl, the `$` settings included a short description of the setting immediately after the value. However, due to flash restrictions, most human-readable strings were removed to free up flash for the new override features in Grbl v1.1. In short, it was these strings or overrides, and overrides won. Keep in mind that once these values are set, they usually don't change, and GUIs will likely provide the assistance of translating these codes for users.

  - _**NOTE for GUI developers:**_ _As with the error and alarm codes, settings codes are available in an easy to parse CSV file in the `/doc/csv` folder. These are continually updated._

  - The `$$` settings print out is shown below and the following describes each setting.

    ```
$0=10
$1=25
$2=0
$3=0
$4=0
$5=0
$6=0
$10=255
$11=0.010
$12=0.002
$13=0
$20=0
$21=0
$22=0
$23=0
$24=25.000
$25=500.000
$26=250
$27=1.000
$30=1000
$31=0
$32=0
$100=250.000
$101=250.000
$102=250.000
$110=500.000
$111=500.000
$112=500.000
$120=10.000
$121=10.000
$122=10.000
$130=200.000
$131=200.000
$132=200.000
ok
```

	| `$x` Code | Setting Description, Units |
|:-------------:|----|
| **`0`** | Step pulse time, microseconds |
| **`1`** | Step idle delay, milliseconds |
| **`2`** | Step pulse invert, mask |
| **`3`** | Step direction invert, mask |
| **`4`** | Invert step enable pin, boolean |
| **`5`** | Invert limit pins, boolean |
| **`6`** | Invert probe pin, boolean |
| **`10`** | Status report options, mask |
| **`11`** | Junction deviation, millimeters |
| **`12`** | Arc tolerance, millimeters |
| **`13`** | Report in inches, boolean |
| **`20`** | Soft limits enable, boolean |
| **`21`** | Hard limits enable, boolean |
| **`22`** | Homing cycle enable, boolean |
| **`23`** | Homing direction invert, mask |
| **`24`** | Homing locate feed rate, mm/min |
| **`25`** | Homing search seek rate, mm/min  |
| **`26`** | Homing switch debounce delay, milliseconds |
| **`27`** | Homing switch pull-off distance, millimeters |
| **`30`** | Maximum spindle speed, RPM |
| **`31`** | Minimum spindle speed, RPM |
| **`32`** | Laser-mode enable, boolean |
| **`100`** | X-axis steps per millimeter |
| **`101`** | Y-axis steps per millimeter |
| **`102`** | Z-axis steps per millimeter |
| **`110`** | X-axis maximum rate, mm/min |
| **`111`** | Y-axis maximum rate, mm/min |
| **`112`** | Z-axis maximum rate, mm/min |
| **`120`** | X-axis acceleration, mm/sec^2 |
| **`121`** | Y-axis acceleration, mm/sec^2 |
| **`122`** | Z-axis acceleration, mm/sec^2 |
| **`130`** | X-axis maximum travel, millimeters |
| **`131`** | Y-axis maximum travel, millimeters |
| **`132`** | Z-axis maximum travel, millimeters |


- The other `$Nx=line` message is the print-out of a user-defined startup line, where `x` denotes the startup line order and ranges from `0` to `1` by default. The `line` denotes the startup line to be executed by Grbl upon reset or power-up, except during an ALARM.

  - When a user queries for the startup lines via a `$N` command, the following is sent by Grbl and completed by an `ok` response. The first line sets the initial startup work coordinate system to `G54`, while the second line is empty and does not execute.
  ```
  $N0=G54
  $N1=
  ok
  ```


------

#### Feedback Messages

Feedback messages provide non-critical information on what Grbl is doing, what it needs, and/or provide some non-real-time data for the user when queried. Not too complicated. Feedback message are always enclosed in `[]` brackets, except for the startup line execution message which begins with an open chevron character `>`.

- **Non-Queried Feedback Messages:** These feedback messages that may appear at any time and is not part of a query are listed and described below. They are usually sent as an additional helpful acknowledgement of some event or command executed. These always start with a `[MSG:` to denote their type.

  - `[MSG:Reset to continue]` - Critical event message. Reset is required before Grbl accepts any other commands. This prevents ongoing command streaming and risking a motion before the alarm is acknowledged. Only hard or soft limit errors send this message immediately after the ALARM:x code.

  - `[MSG:‘$H’|’$X’ to unlock]`- Alarm state is active at initialization. This message serves as a reminder note on how to cancel the alarm state. All g-code commands and some ‘$’ are blocked until the alarm state is cancelled via homing `$H` or unlocking `$X`. Only appears immediately after the `Grbl` welcome message when initialized with an alarm. Startup lines are not executed at initialization if this message is present and the alarm is active.

  - `[MSG:Caution: Unlocked]` - Appears as an alarm unlock `$X` acknowledgement. An 'ok' still appears immediately after to denote the `$X` was parsed and executed. This message reminds the user that Grbl is operating under an unlock state, where startup lines have still not be executed and should be cautious and mindful of what they do. Grbl may not have retained machine position due to an alarm suddenly halting the machine.  A reset or re-homing Grbl is highly recommended as soon as possible, where any startup lines will be properly executed.

  - `[MSG:Enabled]` - Appears as a check-mode `$C` enabled acknowledgement. An 'ok' still appears immediately after to denote the `$C` was parsed and executed.

  - `[MSG:Disabled]` - Appears as a check-mode `$C` disabled acknowledgement. An 'ok' still appears immediately after to denote the `$C` was parsed and executed. Grbl is automatically reset afterwards to restore all default g-code parser states changed by the check-mode.

  - `[MSG:Check Door]` - This message appears whenever the safety door detected as open. This includes immediately upon a safety door switch detects a pin change or appearing after the welcome message, if the safety door is ajar when Grbl initializes after a power-up/reset.

    - If in motion and the safety door switch is triggered, Grbl will immediately send this message, start a feed hold, and place Grbl into a suspend with the DOOR state.
    - If not in motion and not at startup, the same process occurs without the feed hold.
    - If Grbl is in a DOOR state and already suspended, any detected door switch pin detected will generate this message, including a door close.
    - If this message appears at startup, Grbl will suspended into immediately into the DOOR state. The startup lines are executed immediately after the DOOR state is exited by closing the door and sending Grbl a resume command.

  - `[MSG:Check Limits]` - If Grbl detects a limit switch as triggered after a power-up/reset and hard limits are enabled, this will appear as a courtesy message immediately after the Grbl welcome message.

  - `[MSG:Pgm End]` - M2/30 program end message to denote g-code modes have been restored to defaults according to the M2/30 g-code description.

  - `[MSG:Restoring defaults]` - Appears as an acknowledgement message when restoring EEPROM defaults via a `$RST=` command. An 'ok' still appears immediately after to denote the `$RST=` was parsed and executed.
  
  - `[MSG:Sleeping]` - Appears as an acknowledgement message when Grbl's sleep mode is invoked by issuing a `$SLP` command when in IDLE or ALARM states. Note that Grbl-Mega may invoke this at any time when the sleep timer option has been enabled and the timeout has been exceeded. Grbl may only be exited by a reset in the sleep state and will automatically enter an alarm state since the steppers were disabled.
	  - NOTE: Sleep will also invoke the parking motion, if it's enabled. However, if sleep is commanded during an ALARM, Grbl will not park and will simply de-energize everything and go to sleep.

- **Queried Feedback Messages:**

	- `[GC:]` G-code Parser State Message 

		```
		[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0.0 S0]
		ok
		```
		
   		- Initiated by the user via a `$G` command. Grbl replies as follows, where the `[GC:` denotes the message type and is followed by a separate `ok` to confirm the `$G` was executed.
     	
     	- The shown g-code are the current modal states of Grbl's g-code parser. This may not correlate to what is executing since there are usually several motions queued in the planner buffer.
     	
     	- NOTE: Program modal state has been fixed and will show `M0`, `M2`, or `M30` when they are active. During a run state, nothing will appear for program modal state.
     	 


  - `[HLP:]` : Initiated by the user via a `$` print help command. The help message is shown below with a separate `ok` to confirm the `$` was executed.
    ```
    [HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $C $X $H ~ ! ? ctrl-x]
    ok
    ```
		- _**NOTE:** Grbl v1.1's new override real-time commands are not included in the help message. They use the extended-ASCII character set, which are not easily type-able, and require a GUI that supports them. This is for two reasons: Establish enough characters for all of the overrides with extra for later growth, and prevent accidental keystrokes or characters in a g-code file from enacting an override inadvertently._


  - The `$#` print parameter data query produces a large set of data which shown below and completed by an `ok` response message.

    	- Each line of the printout is starts with the data type, a `:`, and followed by the data values. If there is more than one, the order is XYZ axes, separated by commas.

      		```
      [G54:0.000,0.000,0.000]
      [G55:0.000,0.000,0.000]
      [G56:0.000,0.000,0.000]
      [G57:0.000,0.000,0.000]
      [G58:0.000,0.000,0.000]
      [G59:0.000,0.000,0.000]
      [G28:0.000,0.000,0.000]
      [G30:0.000,0.000,0.000]
      [G92:0.000,0.000,0.000]
      [TLO:0.000]
      [PRB:0.000,0.000,0.000:0]
      ok
      ```

    	- The `PRB:` probe parameter message includes an additional `:` and suffix value is a boolean. It denotes whether the last probe cycle was successful or not.

  - `[VER:]` and `[OPT:]`: Indicates build info data from a `$I` user query. These build info messages are followed by an `ok` to confirm the `$I` was executed, like so:
 
      ```
      [VER:v1.1f.20170131:Some string]
      [OPT:VL,16,128]
      ok
      ```
      
  		- The first line `[VER:]` contains the build version and date.
      - A string may appear after the second `:` colon. It is a stored EEPROM string a user via a `$I=line` command or OEM can place there for personal use or tracking purposes.
  		- The `[OPT:]` line follows immediately after and contains character codes for compile-time options that were either enabled or disabled and two values separated by commas, which indicates the total usable planner blocks and serial RX buffer bytes, respectively. The codes are defined below and a CSV file is also provided for quick parsing. This is generally only used for quickly diagnosing firmware bugs or compatibility issues. 

			| `OPT` Code | Setting Description, Units |
|:-------------:|----|
| **`V`** | Variable spindle enabled |
| **`N`** | Line numbers enabled |
| **`M`** | Mist coolant enabled |
| **`C`** | CoreXY enabled |
| **`P`** | Parking motion enabled |
| **`Z`** | Homing force origin enabled |
| **`H`** | Homing single axis enabled |
| **`T`** | Two limit switches on axis enabled |
| **`D`** | Spindle direction pin used as enable pin |
| **`0`** | Spindle enable off when speed is zero enabled |
| **`S`** | Software limit pin debouncing enabled |
| **`R`** | Parking override control enabled |
| **`A`** | Allow feed rate overrides in probe cycles |
| **`+`** | Safety door input pin enabled |
| **`*`** | Restore all EEPROM disabled |
| **`$`** | Restore EEPROM `$` settings disabled |
| **`#`** | Restore EEPROM parameter data disabled |
| **`I`** | Build info write user string disabled |
| **`E`** | Force sync upon EEPROM write disabled |
| **`W`** | Force sync upon work coordinate offset change disabled |
| **`L`** | Homing initialization auto-lock disabled |
    
  - `[echo:]` : Indicates an automated line echo from a command just prior to being parsed and executed. May be enabled only by a config.h option. Often used for debugging communication issues. A typical line echo message is shown below. A separate `ok` will eventually appear to confirm the line has been parsed and executed, but may not be immediate as with any line command containing motions.
      ```
      [echo:G1X0.540Y10.4F100]
      ```
      - NOTE: The echoed line will have been pre-parsed a bit by Grbl. No spaces or comments will appear and all letters will be capitalized.

------

#### Startup Line Execution

  - `>G54G20:ok` : The open chevron indicates a startup line has just executed. The startup line `G54G20` immediately follows the `>` character and is followed by an `:ok` response to indicate it executed successfully.

    - A startup line will execute upon initialization only if a line is present and if Grbl is not in an alarm state.

    - The `:ok` on the same line is intentional. It prevents this `ok` response from being counted as part of a stream, because it is not tied to a sent command, but an internally-generated one.

    - There should always be an appended `:ok` because the startup line is checked for validity before it is stored in EEPROM. In the event that it's not, Grbl will print `>G54G20:error:X`, where `X` is the error code explaining why the startup line failed to execute.

    - In the rare chance that there is an EEPROM read error, the startup line execution will print `>:error:7` with no startup line and a error code `7` for a read fail. Grbl will also automatically wipe and try to restore the problematic EEPROM.


------

#### Real-time Status Reports

- Contains real-time data of Grbl’s state, position, and other data required independently of the stream.

- Categorized as a real-time message, where it is a separate message that should not be counted as part of the streaming protocol. It may appear at any given time.

- A status report is initiated by sending Grbl a '?' character.

  - Like all real-time commands, the '?' character is intercepted and never enters the serial buffer. It's never a part of the stream and can be sent at any time.

  - Grbl will generate and transmit a report within ~5-20 milliseconds.

  - Every ’?’ command sent by a GUI is not guaranteed with a response. The following are the current scenarios when Grbl may not immediately or ignore a status report request. _NOTE: These may change in the future and will be documented here._

    - If two or more '?' queries are sent before the first report is generated, the additional queries are ignored.

    - A soft-reset commanded clears the last status report query.

    - When Grbl throws a critical alarm from a limit violation. A soft-reset is required to resume operation.

    - During a homing cycle.

- **Message Construction:**

  - A message is a single line of ascii text, completed by a carriage return and line feed.

  - `< >` Chevrons uniquely enclose reports to indicate message type.

  - `|` Pipe delimiters separate data fields inside the report.

  - The first data field is an exception to the following data field rules. See 'Machine State' description for details.

  - All remaining data fields consist of a data type followed by a `:` colon delimiter and data values. `type:value(s)`

  - Data values are given either as as one or more pre-defined character codes to indicate certain states/conditions or as numeric values, which are separated by a `,` comma delimiter when more than one is present. Numeric values are also in a pre-defined order and units of measure.

  - The first (Machine State) and second (Current Position) data fields are always included in every report.

  - Assume any following data field may or may not exist and can be in any order. The `$10` status report mask setting can alter what data is present and certain data fields can be reported intermittently (see descriptions for details.)

  - The `$13` report inches settings alters the units of some data values. `$13=0` false indicates mm-mode, while `$13=1` true indicates inch-mode reporting. Keep note of this setting and which report values can be altered.

- **Data Field Descriptions:**

    - **Machine State:**

      - Valid states types:  `Idle, Run, Hold, Jog, Alarm, Door, Check, Home, Sleep`

      - Sub-states may be included via `:` a colon delimiter and numeric code.

      - Current sub-states are:

        	- `Hold:0` Hold complete. Ready to resume.
	     	- `Hold:1` Hold in-progress. Reset will throw an alarm.
        	- `Door:0` Door closed. Ready to resume.
        	- `Door:1` Machine stopped. Door still ajar. Can't resume until closed.
        	- `Door:2` Door opened. Hold (or parking retract) in-progress. Reset will throw an alarm.
			- `Door:3` Door closed and resuming. Restoring from park, if applicable. Reset will throw an alarm.

      - This data field is always present as the first field.

    - **Current Position:**

        - Depending on `$10` status report mask settings, position may be sent as either:

          - `MPos:0.000,-10.000,5.000` machine position or
          - `WPos:-2.500,0.000,11.000` work position

        - **NOTE: Grbl v1.1 sends only one position vector because a GUI can easily compute the other position vector with the work coordinate offset `WCO:` data. See WCO description for details.**

        - Three position values are given in the order of X, Y, and Z. A fourth position value may exist in later versions for the A-axis.

        - `$13` report inches user setting effects these values and is given as either mm or inches.

        - This data field is always present as the second field.

    - **Work Coordinate Offset:**

        - `WCO:0.000,1.551,5.664` is the current work coordinate offset of the g-code parser, which is the sum of the current work coordinate system, G92 offsets, and G43.1 tool length offset.

        - Machine position and work position are related by this simple equation per axis: `WPos = MPos - WCO`
        
        	- **GUI Developers:** Simply track and retain the last `WCO:` vector and use the above equation to compute the other position vector for your position readouts. If Grbl's status reports show either `WPos` or `MPos`, just follow the equations below. It's as easy as that!
        		- If `WPos:` is given, use `MPos = WPos + WCO`.
        		- If `MPos:` is given, use `WPos = MPos - WCO`.

        - Values are given in the order of the X,Y, and Z axes offsets. A fourth offset value may exist in later versions for the A-axis.
        - `$13` report inches user setting effects these values and is given as either mm or inches.

        - `WCO:` values don't change often during a job once set and only requires intermittent refreshing.

        - This data field appears:

          - In every 10 or 30 (configurable 1-255) status reports, depending on if Grbl is in a motion state or not.
          - Immediately in the next report, if an offset value has changed.
          - In the first report after a reset/power-cycle.

        - This data field will not appear if:

          - It is disabled in the config.h file. No `$` mask setting available.
          - The refresh counter is in-between intermittent reports.       

    - **Buffer State:**

        - `Bf:15,128`. The first value is the number of available blocks in the planner buffer and the second is number of available bytes in the serial RX buffer.
        
        - The usage of this data is generally for debugging an interface, but is known to be used to control some GUI-specific tasks. While this is disabled by default, GUIs should expect this data field to appear, but they may ignore it, if desired.
        
        	- IMPORTANT: Do not use this buffer data to control streaming. During a stream, the reported buffer will often be out-dated and may be incorrect by the time it has been received by the GUI. Instead, please use the streaming protocols outlined. They use Grbl's responses as a direct way to accurately determine the buffer state.
        	        
        - NOTE: The buffer state values changed from showing "in-use" blocks or bytes to "available". This change does not require the GUI knowing how many block/bytes Grbl has been compiled with.

        - This data field appears:
        
          - In every status report when enabled. It is disabled in the settings mask by default.
        
        - This data field will not appear if:

          - It is disabled by the `$` status report mask setting or disabled in the config.h file.

    - **Line Number:**

        - `Ln:99999` indicates line 99999 is currently being executed. This differs from the `$G` line `N` value since the parser is usually queued few blocks behind execution.

        - Compile-time option only because of memory requirements. However, if a GUI passes indicator line numbers onto Grbl, it's very useful to determine when Grbl is executing them.

        - This data field will not appear if:

          - It is disabled in the config.h file. No `$` mask setting available.
          - The line number reporting not enabled in config.h. Different option to reporting data field.
          - No line number or `N0` is passed with the g-code block.
          - Grbl is homing, jogging, parking, or performing a system task/motion.
          - There is no motion in the g-code block like a `G4P1` dwell. (May be fixed in later versions.)

    - **Current Feed and Speed:**

        - There are two versions of this data field that Grbl may respond with. 
        
        	- `F:500` contains real-time feed rate data as the value. This appears only when VARIABLE_SPINDLE is disabled in config.h, because spindle speed is not tracked in this mode.
        	- `FS:500,8000` contains real-time feed rate, followed by spindle speed, data as the values. Note the `FS:`, rather than `F:`, data type name indicates spindle speed data is included.
                
        - The current feed rate value is in mm/min or inches/min, depending on the `$` report inches user setting. 
        - The second value is the current spindle speed in RPM
        
        - These values will often not be the programmed feed rate or spindle speed, because several situations can alter or limit them. For example, overrides directly scale the programmed values to a different running value, while machine settings, acceleration profiles, and even the direction traveled can also limit rates to maximum values allowable.

        - As a operational note, reported rate is typically 30-50 msec behind actual position reported.

        - This data field will always appear, unless it was explicitly disabled in the config.h file.

    - **Input Pin State:**

        - `Pn:XYZPDHRS` indicates which input pins Grbl has detected as 'triggered'.

        - Pin state is evaluated every time a status report is generated. All input pin inversions are appropriately applied to determine 'triggered' states.

        - Each letter of `XYZPDHRS` denotes a particular 'triggered' input pin.

          - `X Y Z` XYZ limit pins, respectively
          - `P` the probe pin.
          - `D H R S` the door, hold, soft-reset, and cycle-start pins, respectively.
          - Example: `Pn:PZ` indicates the probe and z-limit pins are 'triggered'.
          - Note: `A` may be added in later versions for an A-axis limit pin.

        - Assume input pin letters are presented in no particular order.

        - One or more 'triggered' pin letter(s) will always be present with a `Pn:` data field.

        - This data field will not appear if:

          - It is disabled in the config.h file. No `$` mask setting available.
          - No input pins are detected as triggered.

    - **Override Values:**

        - `Ov:100,100,100` indicates current override values in percent of programmed values for feed, rapids, and spindle speed, respectively.

        - Override maximum, minimum, and increment sizes are all configurable within config.h. Assume that a user or OEM will alter these based on customized use-cases. Recommend not hard-coding these values into a GUI, but rather just show the actual override values and generic increment buttons.

        - Override values don't change often during a job once set and only requires intermittent refreshing. This data field appears:

          - After 10 or 20 (configurable 1-255) status reports, depending on is in a motion state or not.
          - If an override value has changed, this data field will appear immediately in the next report. However, if `WCO:` is present, this data field will be delayed one report.
          - In the second report after a reset/power-cycle.

        - This data field will not appear if:

          - It is disabled in the config.h file. No `$` mask setting available.
          - The override refresh counter is in-between intermittent reports.
          - `WCO:` exists in current report during refresh. Automatically set to try again on next report.

	- **Accessory State:**

		- `A:SFM` indicates the current state of accessory machine components, such as the spindle and coolant.

		- Due to the new toggle overrides, these machine components may not be running according to the g-code program. This data is provided to ensure the user knows exactly what Grbl is doing at any given time.
		
		- Each letter after `A:` denotes a particular state. When it appears, the state is enabled. When it does not appear, the state is disabled.

			- `S` indicates spindle is enabled in the CW direction. This does not appear with `C`.
			- `C` indicates spindle is enabled in the CCW direction. This does not appear with `S`.
        	- `F` indicates flood coolant is enabled.
        	- `M` indicates mist coolant is enabled.
		
	   - Assume accessory state letters are presented in no particular order.
      
      - This data field appears:

			- When any accessory state is enabled.
        	- Only with the override values field in the same message. Any accessory state change will trigger the accessory state and override values fields to be shown on the next report.

      - This data field will not appear if:

        	- No accessory state is active.
        	- It is disabled in the config.h file. No `$` mask setting available.
        	- If override refresh counter is in-between intermittent reports.
        	- `WCO:` exists in current report during refresh. Automatically set to try again on next report.
