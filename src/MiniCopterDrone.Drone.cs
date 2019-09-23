using System;
using System.Linq;
using System.Collections.Generic;

namespace Oxide.Plugins
{
    public partial class MiniCopterDrone
    {
        //unpartialify:begin Drone
        class Drone {
            public DroneManager manager;
            public EntityRef miniCopterRef;
            public int instanceId;
            public DroneCPU cpu = new DroneCPU();
            public StorageContainer storage = null;

            public PIDController.Angular cyclicController = new PIDController.Angular {
                p = 0.5f,
                i = 0,
                d = 1,
                dRC = 0.5f
            };

            public PIDController.Linear throttleController = new PIDController.Linear {
                p = 1f,
                i = 0,
                d = 1f,
                spRC = 1f
            };

            public PIDController.Vector headingController = new PIDController.Vector {
                p = 1f,
                i = 0,
                d = 2.5f,
                spRC = 1.0f,
                dRC = 0.5f
            };

            [Flags]
            public enum Flag {
                EngineOn = 1 << 0,
                EngineStarting = 1 << 1,
                HasTarget = 1 << 2,
                HasTargetAltitude = 1 << 3,
                Flying = 1 << 4,
                ExecutingInstruction = 1 << 5,
                Landing = 1 << 6,
                SeekingTarget = 1 << 7,
                SeekingTargetFlythrough = 1 << 8,
                SeekingTargetAltitude = 1 << 9
            }

            public Flag flags;

            List<Vector3> targetStack = new List<Vector3>();
            List<float> targetAltStack = new List<float>();

            bool HasFlag(Flag f) {
                return (this.flags & f) == f;
            }

            public void SetFlag(Flag f, bool set) {
                if(set) {
                    this.flags |= f;
                    return;
                }

                this.flags &= ~f;
            }

            public bool active = false;
            Compiler.Instruction currentInstruction = null;
            float desiredAltitude = 0;
            Vector3 target = Vector3.zero;
            float sleepTime = -1;
            float engineStartTime = -1;
            float landingSpeed = 0;
            float landingTime;
            float desiredPitch = 0;
            int waitRF;

            public void Reset() {
                active = false;
                currentInstruction = null;
                desiredAltitude = 0;
                target = Vector3.zero;
                sleepTime = -1;
                flags = 0;
                landingSpeed = 0;
                desiredPitch = 0;

                var copter = miniCopterRef.Get(true) as MiniCopter;

                engineStartTime = -1;

                if(copter) {
                    StopEngine();
                    UpdateEngine();
                }

                targetStack.Clear();
                targetAltStack.Clear();
                
                cpu.Reset();
            }

            float GetTerrainHeight(Vector3 position) {
                return Mathf.Max(TerrainMeta.HeightMap.GetHeight(position), TerrainMeta.WaterMap.GetHeight(position));
            }

            float GetAltitude(Vector3 position) {
                return position.y - GetTerrainHeight(position);
            }

            public bool OnItemAddedOrRemoved(StorageContainer storage, Item item, bool added) {
                if(item.info.shortname == "note" && item.text != null) {
                    var lines = item.text.ToLower().Split(new[] {"\r\n", "\r", "\n", ";"}, StringSplitOptions.RemoveEmptyEntries);

                    if(lines.Length >= 1) {
                        if(lines[0] == "#droneasm") {
                            if(added) {
                                if(lines.Length < config.maxProgramInstructions) {
                                    Reset();
                                    bool success = compiler.Compile(item.text);

                                    if(!success) { 
                                        item.text += "\n" + String.Join("\n", compiler.errors.Select(x => "#[error] " + x));
                                    } else {
                                        cpu.LoadInstructions(compiler.instructions);
                                        active = true;
                                    }
                                } else {
                                    item.text += "\n" + $"#[error] max program length is {config.maxProgramInstructions} instructions";
                                }
                            } else {
                                Reset();
                            }
                        }
                    }
                }

                return true;
            }

            void ControlAltitude(float desiredAltitude, float currentAltitude) {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                var throttleControl = throttleController.Update(desiredAltitude, currentAltitude, Time.fixedDeltaTime);
                float bias = (copter.rigidBody.mass * Physics.gravity.magnitude) / copter.engineThrustMax;
                throttleControl = Mathf.Clamp(throttleControl + bias, -1.0f, 1.0f);

                Vector3 copterUp = copter.transform.rotation * Vector3.up;
                Vector3 thrust = copterUp * (copter.engineThrustMax * throttleControl);

                copter.rigidBody.AddForce(thrust, ForceMode.Force);
            }

            void CyclicControl(Vector3 desired) {
                var copter = miniCopterRef.Get(true) as MiniCopter;
                var pv = copter.transform.rotation * Vector3.up;
                var control = cyclicController.Update(desired, pv, Time.fixedDeltaTime);

                if(control.magnitude > 1e-6f) {
                    copter.rigidBody.AddTorque(control, ForceMode.Acceleration);
                }
            }

            Vector3 HeadingControl(Vector3 target, float desiredPitch) {
                var copter = miniCopterRef.Get(true) as MiniCopter;
                
                var headingControl = headingController.Update(target, copter.transform.position, Time.fixedDeltaTime);
                float pitchAngle = Mathf.Clamp(headingControl.magnitude, -desiredPitch, desiredPitch);
                headingControl = headingControl.normalized * pitchAngle;

                var axis = Vector3.Cross(Vector3.up, headingControl).normalized;
                var rot = Quaternion.AngleAxis(pitchAngle, axis);

                return rot * Vector3.up;
            }

            void TailFinTorque() {
                var copter = miniCopterRef.Get(true) as MiniCopter;
                var tailForce = Vector3.Project(copter.GetWorldVelocity(), copter.transform.right);
                var dot = Vector3.Dot(copter.GetWorldVelocity(), copter.transform.right);
                tailForce *= tailForce.magnitude;
                copter.rigidBody.AddRelativeTorque(new Vector3(0, tailForce.magnitude * Mathf.Sign(dot), 0), ForceMode.Force);
            }

            void StartEngine() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    return;
                }

                if(!HasFlag(Flag.EngineOn) && !HasFlag(Flag.EngineStarting) && !copter.Waterlogged()) {
                    engineStartTime = Time.fixedTime;
                    SetFlag(Flag.EngineStarting, true);
                    copter.EngineStartup();
                }
            }

            void StopEngine() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    return;
                }

                SetFlag(Flag.EngineOn, false);
                SetFlag(Flag.EngineStarting, false);
                copter.EngineOff();
            }

            void UpdateEngine() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    return;
                }

                if(!copter.HasFuel()) {
                    //StopEngine();
                }

                if(HasFlag(Flag.EngineStarting) && Time.fixedTime > engineStartTime + 5.0f) {
                    SetFlag(Flag.EngineOn, true);
                    SetFlag(Flag.EngineStarting, false);
                }

                if(HasFlag(Flag.EngineOn)) {
                    copter.UseFuel(Time.fixedDeltaTime);

                    if(!copter.HasFlag(MiniCopter.Flag_EngineStart)) {
                        copter.SetFlag(MiniCopter.Flag_EngineStart, true);
                    }
                }
            }

            void ResetControl() {
                headingController.Reset();
                throttleController.Reset();
                cyclicController.Reset();
            }

            void Fly() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    return;
                }

                var currentAltitude = GetAltitude(copter.transform.position);

                if(HasFlag(Flag.Landing)) {
                    this.desiredAltitude = Mathf.Max(0, this.desiredAltitude - (this.landingSpeed * Time.fixedDeltaTime));
                }

                this.target.y = this.desiredAltitude + GetTerrainHeight(this.target);

                if(HasFlag(Flag.EngineOn)) {
                    if(HasFlag(Flag.Flying)) {
                        if(HasFlag(Flag.HasTargetAltitude)) {
                            ControlAltitude(desiredAltitude, currentAltitude);
                        }

                        Vector3 copterUp = Vector3.up;

                        if(HasFlag(Flag.HasTarget)) {
                            copterUp = HeadingControl(this.target, this.desiredPitch);
                        }

                        CyclicControl(copterUp);
                    }
                }
            }

            bool TryGetRFPosition(int freq, Vector3 closestTo, out Vector3 position) {
                var list = new List<IRFObject>();

                if(RFManager._broadcasters.TryGetValue(freq, out list)) {
                    if(list.Count == 0) {
                        position = Vector3.zero;
                        return false;
                    }

                    float closest = float.MaxValue;
                    Vector3 pos = Vector3.zero;

                    foreach(var irfObject in list) {
                        float dist = (irfObject.GetPosition() - closestTo).magnitude;

                        if(dist <= irfObject.GetMaxRange()) {
                            if(dist < closest) {
                                pos = irfObject.GetPosition();
                                closest = dist;
                            }
                        }
                    }

                    if(closest != float.MaxValue) {
                        position = pos;
                        return true;
                    }
                }

                position = Vector3.zero;
                return false;
            }

            public void FixedUpdate() {
                MiniCopter copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    Reset();
                    return;
                }

                UpdateEngine();
                TailFinTorque();

                if(!active) {
                    StopEngine();
                    return;
                }

                Fly();

                var currentAltitude = GetAltitude(copter.transform.position);
                var currentPositionTerrainHeight = GetTerrainHeight(copter.transform.position);

                foreach(var freq in DroneManager.risingEdgeFrequencies) {
                    string isrName = "rfa" + freq;
                    cpu.Interrupt(isrName);
                }

                foreach(var freq in DroneManager.fallingEdgeFrequencies) {
                    string isrName = "rfna" + freq;
                    cpu.Interrupt(isrName);
                }

                if(HasFlag(Flag.Landing)) {
                    if(currentAltitude < 2 || (Time.fixedTime - this.landingTime > 5.0f && copter.rigidBody.velocity.magnitude < 0.1f)) {
                        SetFlag(Flag.Landing, false);
                        SetFlag(Flag.Flying, false);
                        cpu.Interrupt("landed");
                    }
                }

                if(HasFlag(Flag.HasTarget) && HasFlag(Flag.SeekingTarget)) {
                    Vector3 offset = this.target - copter.transform.position;

                    if(offset.magnitude < 1 && copter.rigidBody.velocity.magnitude < 1) {
                        SetFlag(Flag.SeekingTarget, false);
                        cpu.Interrupt("at_target");
                    }
                }

                if(HasFlag(Flag.HasTarget) && HasFlag(Flag.SeekingTargetFlythrough)) {
                    Vector3 offset = this.target - copter.transform.position;

                    if(offset.magnitude < 10.0f) {
                        SetFlag(Flag.SeekingTargetFlythrough, false);
                        cpu.Interrupt("almost_at_target");
                    }
                }

                if(HasFlag(Flag.HasTargetAltitude) && HasFlag(Flag.SeekingTargetAltitude)) {
                    if(Mathf.Abs(currentAltitude - this.desiredAltitude) < 1 && copter.rigidBody.velocity.magnitude < 1) {
                        SetFlag(Flag.SeekingTargetAltitude, false);
                        cpu.Interrupt("at_altitude");
                    }
                }

                // this checks if the executing instruction has finished
                if(this.currentInstruction != null && HasFlag(Flag.ExecutingInstruction)) {
                    switch(this.currentInstruction.name) {
                        case "sleep":
                            if(Time.fixedTime > this.sleepTime) {
                                this.sleepTime = -1;
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                        case "startengine":
                            if(HasFlag(Flag.EngineOn)) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                        case "waitrf":
                            if(DroneManager.activeFrequencies.Contains(this.waitRF)) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                        case "land":
                            if(!HasFlag(Flag.Landing)) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                        case "flyto":
                            if(!HasFlag(Flag.SeekingTarget)) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                        case "flythrough":
                            if(!HasFlag(Flag.SeekingTargetFlythrough)) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }

                            break;
                    }
                }

                if(cpu.HandlePendingInterrupt()) {
                    SetFlag(Flag.ExecutingInstruction, false);
                }

                int numInstructionsExecuted = 0;
                string failReason;
                while(numInstructionsExecuted < config.maxInstructionsPerFixedUpdate && !HasFlag(Flag.ExecutingInstruction)) {
                    // these need to stay synced
                    this.target.y = this.desiredAltitude + GetTerrainHeight(this.target);

                    if(!cpu.Cycle(out this.currentInstruction, out failReason)) {
                        Reset();
                        return;
                    }

                    numInstructionsExecuted++;

                    switch(this.currentInstruction.name) {
                        case "sleep":
                            var seconds = currentInstruction.args[0].floatValue;
                            
                            if(this.sleepTime == -1) {
                                this.sleepTime = Time.fixedTime + seconds;
                            }
                            
                            SetFlag(Flag.ExecutingInstruction, true);
                            break;
                        case "startengine":
                            if(!HasFlag(Flag.EngineStarting)) {
                                StartEngine();
                                SetFlag(Flag.ExecutingInstruction, true);
                            }

                            break;
                        case "stopengine":
                            StopEngine();
                            break;
                        case "target":
                            var mapCol = currentInstruction.args[0].floatValue;
                            var mapRow = currentInstruction.args[1].floatValue;
                            var desiredAltitude = currentInstruction.args[2].floatValue;

                            Vector3 pos1;
                            pos1 = MapGridToPosition(mapCol, mapRow);
                            this.target = pos1;
                            this.desiredAltitude = desiredAltitude;
                            SetFlag(Flag.HasTarget, true);
                            SetFlag(Flag.HasTargetAltitude, true);
                            ResetControl();

                            break;
                        case "targetalt":
                            this.desiredAltitude = currentInstruction.args[0].floatValue;
                            SetFlag(Flag.HasTargetAltitude, true);
                            ResetControl();
                            break;
                        case "targetrf":
                            var freq = currentInstruction.args[0].intValue;

                            Vector3 pos2;
                            if(TryGetRFPosition(freq, copter.transform.position, out pos2)) {
                                this.target = pos2;
                                this.desiredAltitude = this.target.y - GetTerrainHeight(this.target);
                                SetFlag(Flag.HasTarget, true);
                                SetFlag(Flag.HasTargetAltitude, true);
                                ResetControl();
                            }
                            
                            break;
                        case "targethere":
                            this.target = copter.transform.position;
                            this.desiredAltitude = this.target.y - GetTerrainHeight(this.target);
                            SetFlag(Flag.HasTarget, true);
                            SetFlag(Flag.HasTargetAltitude, true);
                            ResetControl();
                            break;
                        case "land":
                            var speed = currentInstruction.args[0].floatValue;

                            if(HasFlag(Flag.Flying)) {
                                this.landingSpeed = speed;
                                this.landingTime = Time.fixedTime;
                                SetFlag(Flag.Landing, true);
                                SetFlag(Flag.ExecutingInstruction, true);
                            }

                            break;
                        case "flyto":
                        case "flythrough":
                            if(HasFlag(Flag.HasTarget) || HasFlag(Flag.HasTargetAltitude)) {
                                SetFlag(Flag.Flying, true);
                                SetFlag(Flag.ExecutingInstruction, true);

                                if(HasFlag(Flag.HasTarget)) {
                                    SetFlag(Flag.SeekingTarget, true);
                                }

                                if(HasFlag(Flag.HasTargetAltitude)) {
                                    SetFlag(Flag.SeekingTargetAltitude, false);
                                }
                            }

                            break;
                        case "drop":
                            var slot = currentInstruction.args[0].intValue;

                            for(int i = 0; i < storage.inventory.itemList.Count; i++) {
                                var item = storage.inventory.itemList[i];
                                
                                if(item.position == slot) {
                                    Vector3 randomVector = new Vector3(UnityEngine.Random.Range(-0.5f, 0.5f), 0, UnityEngine.Random.Range(-0.5f, 0.5f));
                                    item.Drop(storage.GetDropPosition() + Vector3.down * 2 + randomVector, storage.GetDropVelocity());
                                }
                            }

                            break;
                        case "pushtarget":
                            targetStack.Add(this.target);
                            
                            if(targetStack.Count > 32) {
                                Reset();
                                return;
                            }
                            
                            break;
                        case "poptarget":                           
                            if(targetStack.Count == 0) {
                                Reset();
                                return;
                            }

                            this.target = targetStack[targetStack.Count - 1];
                            targetStack.RemoveAt(targetStack.Count - 1);
                            ResetControl();
                            
                            break;
                        case "pushtargetalt":
                            targetAltStack.Add(this.desiredAltitude);
                            
                            if(targetAltStack.Count > 32) {
                                Reset();
                                return;
                            }
                            
                            break;
                        case "poptargetalt":                           
                            if(targetAltStack.Count == 0) {
                                Reset();
                                return;
                            }

                            this.desiredAltitude = targetAltStack[targetAltStack.Count - 1];
                            targetAltStack.RemoveAt(targetAltStack.Count - 1);
                            ResetControl();
                            
                            break;
                        case "gettarget":
                            var stackPos = this.currentInstruction.args[0].intValue;
                            var xVar = this.currentInstruction.args[1].stringValue;
                            var yVar = this.currentInstruction.args[2].stringValue;
                            var zVar = this.currentInstruction.args[3].stringValue;
                            Vector3 targetToUse = Vector3.zero;

                            if(stackPos == -1) {
                                targetToUse = this.target;
                            } else {
                                if(this.targetStack.Count >= stackPos + 1) {
                                    targetToUse = this.targetStack[stackPos];
                                }
                            }

                            cpu.WriteVariable(xVar, targetToUse.x);
                            cpu.WriteVariable(yVar, targetToUse.y);
                            cpu.WriteVariable(zVar, targetToUse.z);
                            break;
                        case "getalt":
                            var stackPos2 = this.currentInstruction.args[0].intValue;
                            var altVar = this.currentInstruction.args[1].stringValue;
                            float altToUse = 0;

                            if(stackPos2 == -1) {
                                altToUse = this.desiredAltitude;
                            } else {
                                if(this.targetAltStack.Count >= stackPos2 + 1) {
                                    altToUse = this.targetAltStack[stackPos2];
                                }
                            }

                            cpu.WriteVariable(altVar, altToUse);
                            break;
                        case "settarget":
                            var stackPos3 = this.currentInstruction.args[0].intValue;
                            var xVal = this.currentInstruction.args[1].floatValue;
                            var yVal = this.currentInstruction.args[2].floatValue;
                            var zVal = this.currentInstruction.args[3].floatValue;

                            if(stackPos3 == -1) {
                                this.target = new Vector3(xVal, yVal, zVal);
                                this.desiredAltitude = this.target.y - GetTerrainHeight(this.target);
                                SetFlag(Flag.HasTarget, true);
                                SetFlag(Flag.HasTargetAltitude, true);
                                ResetControl();
                            } else {
                                if(this.targetStack.Count >= stackPos3 + 1) {
                                    this.targetStack[stackPos3] = new Vector3(xVal, yVal, zVal);
                                    this.desiredAltitude = this.target.y - GetTerrainHeight(this.target);
                                    SetFlag(Flag.HasTarget, true);
                                    SetFlag(Flag.HasTargetAltitude, true);
                                    ResetControl();
                                }
                            }

                            break;
                        case "setalt":
                            var stackPos4 = this.currentInstruction.args[0].intValue;
                            float altVal = this.currentInstruction.args[1].floatValue;

                            if(stackPos4 == -1) {
                                this.desiredAltitude = altVal;
                                SetFlag(Flag.HasTargetAltitude, true);
                                ResetControl();
                            } else {
                                if(this.targetAltStack.Count >= stackPos4 + 1) {
                                    this.targetAltStack[stackPos4] = altVal;
                                    SetFlag(Flag.HasTargetAltitude, true);
                                    ResetControl();
                                }
                            }

                            break;
                        case "setpitch":
                            this.desiredPitch = this.currentInstruction.args[0].floatValue;
                            break;
                        case "fly":
                            if(HasFlag(Flag.HasTarget) || HasFlag(Flag.HasTargetAltitude)) {
                                SetFlag(Flag.Flying, true);

                                if(HasFlag(Flag.HasTarget)) {
                                    SetFlag(Flag.SeekingTarget, true);
                                }

                                if(HasFlag(Flag.HasTargetAltitude)) {
                                    SetFlag(Flag.SeekingTargetAltitude, false);
                                }
                            }

                            break;
                        case "descend":
                            var speed2 = currentInstruction.args[0].floatValue;

                            if(HasFlag(Flag.Flying)) {
                                this.landingSpeed = speed2;
                                this.landingTime = Time.fixedTime;
                                SetFlag(Flag.Landing, true);
                            }

                            break;
                        case "waitrf":
                            this.waitRF = currentInstruction.args[0].intValue;
                            SetFlag(Flag.ExecutingInstruction, true);
                            break;
                    }
                }
            }
        }
        //unpartialify:end
    }
}