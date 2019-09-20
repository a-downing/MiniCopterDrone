# MiniCopterDrone
### A Rust uMod plugin giving the minicopter drone functionality
**(This guide is not even close to complete)**

To activate the drone drag a note containing instructions into the wooden box (may be invisible) under the tail.

#### Note format:
The first line must be #droneasm, instructions are separated by newlines or a semicolon, comments start with # and continue to the end of the line
Example:
```
#droneasm
startengine #starting the engine
```
  
 #### Basic instructions:

`startengine`
___

`stopengine`
___

`target <map_col> <map_row> <altitude_in_meters>`

targets a point of the map. For example to target the center of grid A0 50 meters above the terrain:
`target A.5 0.5 50`
___

`sleep <seconds>`

Pause execution for the given number of seconds (interrupts can still be caught)
___
`targetalt <altitude_in_meters>`

changes the target altitude, leaving the targeted ground position if any unchanged
___    
`targethere`

target the current position and altitude of the minicopter
___
`targetrf <frequency>`

target the position of the closest broadcasting transmitter on the given frequency, if none, the current target is unaffected
___
`flyto <pitch_angle>`

Fly to the current target at the provided pitch angle. The instruction finishes when the target is reached
___
`flythrough <pitch_angle>`

Fly to the current target at the provided pitch angle. The instruction finishes when 10 meters from the target
___
`land <speed>`

Land at `<speed>` meters per second
___
`drop <slot_number>`

Drop the item in slot number `<slot_number>` (in the wooden box). The top left slot is 0, botton right is 11
___
`waitrf <frequency>`

Pause until an RF broadcaster transmits on frequency `<frequency>`
___

#### Advanced Instructions
These instructions don't directly affect the flight of the minicopter, but are for program control flow

`label <label_name>`

Place a label on a line to be able to jump to it
___
`isr <interrupt_name>`

Label for CPU to jump to when a specific interrupt is triggered
___
`jmp <label_name>`

Jump to a label
___
`call <label_name>`

Jump to a label being able to `ret` afterwards
___
`int <interrupt_name>`

Trigger an interrupt from software
___
`ret`

Return from a `call` or `int`

#### Interrupts
`rfa<frequency>`

Triggered when an RF broadcaster starts transmitting on the given frequency, for example `isr rfa777` or `int rfa777`
___
`rfna<frequency>`

Triggered when an RF broadcaster stops transmitting on the given frequency


#### Examples

Fly to H 7 at an altitude of 100 meters and maximum pitch angle of 20 degrees, land at 5 meters per second, wait for 1 minute, then fly to J13

```
#droneasm
startengine
target H 7 100
flyto 20
land 5
sleep 60
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
    targethere
    targetalt 50 #target 50 meters above the current position
    flyto 30 # fly to the target (it will continue to hover there until given another instruction)
    
    label loop
        jmp loop #loop forever, if the program finishes the drone shuts down
```
