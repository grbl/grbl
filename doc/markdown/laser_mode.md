## Grbl v1.1 Laser Mode

**_DISCLAIMER: Lasers are extremely dangerous devices. They can instantly cause fires and permanently damage your vision. Please read and understand all related documentation for your laser prior to using it. The Grbl project is not resposible for any damage or issues the firmware may cause, as defined by its GPL license._**

----

Outlined in this document is how Grbl alters its running conditions for the new laser mode to provide both improved performance and enforcing some basic user safety precautions.

## Laser Mode Overview

The main difference between default Grbl operation and the laser mode is how the spindle/laser output is controlled with motions involved. Every time a spindle state `M3 M4 M5` or spindle speed `Sxxx` is altered, Grbl would come to a stop, allow the spindle to change, and then continue. This is the normal operating procedure for a milling machine spindle. It needs time to change speeds. 

However, if a laser starts and stops like this for every spindle change, this leads to scorching and uneven cutting/engraving! Grbl's new laser mode prevents unnecessary stops whenever possible and adds a new dynamic laser power mode that automagically scales power based on current speed related to programmed rate. So, you can get super clean and crisp results, even on a low-acceleration machine!

Enabling or disabling Grbl's laser mode is easy. Just alter the **$32** Grbl setting. 
- **To Enable**: Send Grbl a `$32=1` command. 
- **To Disable:** Send Grbl a `$32=0` command.

**WARNING:** If you switch back from laser mode to a spindle for milling, you **MUST** disable laser mode by sending Grbl a `$32=0` command. Milling operations require the spindle to get up to the right rpm to cut correctly and to be **safe**, helping to prevent a tool from breaking and flinging metal shards everywhere. With laser mode disabled, Grbl will briefly pause upon any spindle speed or state change to give the spindle a chance to get up to speed before continuing.


## Laser Mode Operation

When laser mode is enabled, Grbl controls laser power by varying the **0-5V** voltage from the spindle PWM D11 pin. **0V** should be treated as disabled, while **5V** is full power. Intermediate output voltages are also assumed to be linear with laser power, such that **2.5V** is approximate 50% laser power. (A compile time option exists to shift this linear model to start at a non-zero voltage.) 

By default, the spindle PWM frequency is **1kHz**, which is the recommended PWM frequency for most current Grbl-compatible lasers system. If a different frequency is required, this may be altered by editing the `cpu_map.h` file. 

The laser is enabled with the `M3` spindle CW and `M4` spindle CCW commands. These enable two different laser modes that are advantageous for different reasons each.
	
- **`M3` Constant Laser Power Mode:**

    - Constant laser power mode simply keeps the laser power as programmed, regardless if the machine is moving, accelerating, or stopped. This provides better control of the laser state. With a good g-code program, this can lead to more consistent cuts in more difficult materials. 
    
    - For a clean cut and prevent scorching with `M3` constant power mode, it's a good idea to add lead-in and lead-out motions around the line you want to cut to give some space for the machine to accelerate and decelerate.

    - NOTE: `M3` can be used to keep the laser on for focusing.

- **`M4` Dynamic Laser Power Mode:**
    - Dynamic laser power mode will automatically adjust laser power based on the current speed relative to the programmed rate. It'll essentially ensures the amount of laser energy along a cut is consistent even though the machine may be stopped or actively accelerating. This is very useful for clean, precise engraving and cutting on simple materials across a large range of g-code generation methods by CAM programs. It will generally run faster and may be all you need to use.
    
    - Grbl calculates laser power based on the assumption that  laser power is linear with speed and the material. Often, this is not the case. Lasers can cut differently at varying power levels and some materials may not cut well at a particular speed and/power. In short, this means that dynamic power mode may not work for all situations. Always do a test piece prior to using this with a new material or machine.
		
    - When not in motion, `M4` dynamic mode turns off the laser. It only turns on when the machine moves. This generally makes the laser safer to operate, because, unlike `M3`, it will never burn a hole through your table, if you stop and forget to turn `M3` off in time.

Describe below are the operational changes to Grbl when laser mode is enabled. Please read these carefully and understand them fully, because nothing is worse than a garage _**fire**_.

- Grbl will move continuously through **consecutive** motion commands when programmed with a new `S` spindle speed (laser power). The spindle PWM pin will be updated instantaneously through each motion without stopping.
	- Example: The following set of g-code commands will not pause between each of them when laser mode is enabled, but will pause when disabled.
	
		```
		G1 X10 S100 F50
		G1 X0 S90
		G2 X0 I5 S80
		``` 
	- Grbl will enforce a laser mode motion stop in a few circumstances. Primarily to ensure alterations stay in sync with the g-code program.

		- Any `M3`, `M4`, `M5` spindle state _change_. 
		- `M3` only and no motion programmed: A `S` spindle speed _change_.
		- `M3` only and no motion programmed: A `G1 G2 G3` laser powered state _change_ to `G0 G80` laser disabled state.
		- NOTE: `M4` does not stop for anything but a spindle state _change_.

- The laser will only turn on when Grbl is in a `G1`, `G2`, or `G3` motion mode. 

	- In other words, a `G0` rapid motion mode or `G38.x` probe cycle will never turn on and always disable the laser, but will still update the running modal state. When changed to a `G1 G2 G3` modal state, Grbl will immediately enable the laser based on the current running state.
	
	- Please remember that `G0` is the default motion mode upon power up and reset. You will need to alter it to `G1`, `G2`, or `G3` if you want to manually turn on your laser. This is strictly a safety measure.
	
	- Example: `G0 M3 S1000` will not turn on the laser, but will set the laser modal state to `M3` enabled and power of `S1000`. A following `G1` command will then immediately be set to `M3` and `S1000`.

	- To have the laser powered during a jog motion, first enable a valid motion mode and spindle state. The following jog motions will inherit and maintain the previous laser state. Please use with caution though. This ability is primarily to allow turning on the laser on a _very low_ power to use the laser dot to jog and visibly locate the start position of a job.


- A `S0` spindle speed of zero will turn off the laser. When programmed with a valid laser motion, Grbl will disable the laser instantaneously without stopping for the duration of that motion and future motions until set greater than zero..

	- `M3` constant laser mode, this is a great way to turn off the laser power while continuously moving between a `G1` laser motion and a `G0` rapid motion without having to stop. Program a short `G1 S0` motion right before the `G0` motion and a `G1 Sxxx` motion is commanded right after to go back to cutting.


-----
###CAM Developer Implementation Notes

TODO: Add some suggestions on how to write laser g-code for Grbl. 

- When using `M3` constant laser power mode, try to avoid force-sync conditions during a job whenever possible. Basically every spindle speed change must be accompanied by a valid motion. Any motion is fine, since Grbl will automatically enable and disable the laser based on the modal state. Avoid a `G0` and `G1` command with no axis words in this mode and in the middle of a job.

- Ensure smooth motions throughout by turning the laser on and off without an `M3 M4 M5` spindle state command. There are two ways to do this:

    - _Program a zero spindle speed `S0`_: `S0` is valid g-code and turns off the spindle/laser without changing the spindle state. In laser mode, Grbl will smoothly move through consecutive motions and turn off the spindle. Conversely, you can turn on the laser with a spindle speed `S` greater than zero. Remember that `M3` constant power mode requires any spindle speed `S` change to be programmed with a motion to allow continuous motion, while `M4` dynamic power mode does not.

    - _Program an unpowered motion between powered motions_: If you are traversing between parts of a raster job that don't need to have the laser powered, program a `G0` rapid between them. `G0` enforces the laser to be disabled automatically. The last spindle speed programmed doesn't change, so if a valid powered motion, like a `G1` is executed after, it'll immediately re-power the laser with the last programmed spindle speed when executing that motion.
