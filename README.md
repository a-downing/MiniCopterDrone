# MiniCopterDrone
### A Rust uMod plugin giving the minicopter drone functionality


#### Configuration:
`"gridPositionCorrection"`:  (see console commands)

`"gridSize"`: The length of a map grid square in units of game coordinates, you shouldn't need to change this.

`"maxInstructionsPerFixedUpdate"`: The maximum number of instructions a drone can execute per FixedUpdate().

`"maxProgramInstructions"`: The maximum number of instructions in a drone program

#### Console Commands:
`minicopterdrone.calibrate <map_col><map_row>`: This calibrates the relation between map grid lines and actual game coordinates and saves it to the `"gridPositionCorrection"` config property. To use it move your player to a grid line intersection, and give the coordinate as the argument, for example: `minicopterdrone.calibrate A13`. This may need to be run every time the map size is changed.

#### Permissions: 
`minicopterdrone.calibrate.allowed`: This is needed to run the `minicopterdrone.calibrate <map_col><map_row>` console command.


#### Use:

**To activate the drone drag a note containing instructions into the wooden box (may be invisible) under the tail. The minicopter must have fuel.**

**Note format:**
The first line must be #droneasm, instructions are separated by newlines or a semicolon, comments start with # and continue to the end of the line 

**Basic instructions:**

`startengine`

start the engine

`stopengine`

stop the engine

`target <map_col> <map_row> <altitude_in_meters>`

targets a point of the map. For example to target the center of grid A0 50 meters above the terrain: `target A.5 0.5 50`

`sleep <seconds>`

pause execution for the given number of seconds (interrupts can still be caught)

`targetalt <altitude_in_meters>`

changes the target altitude, leaving the targeted ground position if any unchanged

`targethere`

target the current position and altitude of the minicopter

`targetrf <frequency>`

target the position of the closest broadcasting transmitter on the given frequency, if none, the current target is unaffected

`pushtarget` / `pushtargetalt`

push the current target/alt onto a stack

`poptarget` / `poptargetalt`

remove the most recently pushed target/alt and set it to the current target 

`setpitch <pitch>`

set the maximum pitch angle

`flyto <pitch_angle>`

fly to the current target. The instruction finishes when the target is reached

`flythrough <pitch_angle>`

fly to the current target. The instruction finishes when 10 meters from the target

`land <speed>`

land at `<speed>` meters per second

`drop <slot_number>`

drop the item in slot number `<slot_number>` (in the wooden box). The top left slot is 0, botton right is 11

`waitrf <frequency>`

pause until an RF broadcaster transmits on frequency `<frequency>`

**Advanced Instructions:**
These instructions don't directly affect the flight of the minicopter, but are for program control flow

`label <label_name>`

place a label on a line to be able to jump to it

`isr <interrupt_name>`

label for drone CPU to jump to when a specific interrupt is triggered

`jmp <label_name>`

jump to a label

`call <label_name>`

jump to a label being able to ret afterwards

`int <interrupt_name>`

trigger an interrupt from software

`ret`

return from a call or int

**For Very Advanced Users:** (see the code)

registers: `r0` - `r7` and `rslt`

Instructions:

`num`, `mov`, `push`, `pop`, `je`, `jne`, `ja`, `jna`, `jg`, `jge`, `jl`, `jle`, `add`, `sub`, `mul`, `div`, `sqrt`, `pow`, `round`, `floor`, `ceil`, `min`, `max`, `lerp`

**Interrupts:**

`rfa<frequency>`

triggered when an RF broadcaster starts transmitting on the given frequency, for example `isr rfa777` or `int rfa777`

`rfna<frequency>`

triggered when an RF broadcaster stops transmitting on the given frequency

`at_target`

triggered when a target is reached

`almost_at_target`

triggered when 10 meters from target

`at_altitude`

triggered when at the target altitude

`landed`

triggered when landed

**Examples:**

Fly to H 7 at an altitude of 100 meters and maximum pitch angle of 20 degrees, land at 5 meters per second, wait for RF freq 333, then fly to J13

```
#droneasm
startengine
target H 7 100
setpitch 20
flyto
land 5
waitrf 333
target J 13 100
flyto
land 5
```

Hover 50 meters above the current position. When RF frequency 5555 is transmitted, fly to the position it was broadcast from.

```
#droneasm
jmp main #jump to the main part of the program

isr rfa5555 #program execution jumps here when an RF broadcaster starts broadcasting on 5555
    targetrf 5555 #target the position of the closest broadcaster
    ret #return to the instruction that was executing before the interrupt (the loop)
    
label main
    startengine
    setpitch 30
    targethere
    targetalt 50 #target 50 meters above the current position
    flyto # fly to the target (it will continue to hover there until given another instruction)
    
    label loop
        jmp loop #loop forever, if the program finishes the drone shuts down
```
