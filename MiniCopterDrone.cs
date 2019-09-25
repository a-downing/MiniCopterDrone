using System;
using Oxide.Core;
using Oxide.Core.Configuration;
using Oxide.Core.Plugins;
using Oxide.Core.Libraries.Covalence;
using System.Linq;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Text.RegularExpressions;
using UnityEngine;

namespace Oxide.Plugins
{
<<<<<<< HEAD
    [Info("Minicopter Drone", "Andrew", "1.2.0")]
=======
    [Info("Minicopter Drone", "Andrew", "1.2.1")]
>>>>>>> develop
    public class MiniCopterDrone : RustPlugin {
        static MiniCopterDrone plugin = null;
        DroneManager droneManager = null;
        static ConfigData config;
        static Compiler compiler = null;
        const string calibratePerm = "minicopterdrone.calibrate.allowed";

        class ConfigData {
            [JsonProperty(PropertyName = "maxProgramInstructions")]
            public int maxProgramInstructions;
            [JsonProperty(PropertyName = "gridPositionCorrection")]
            public Vector3 gridPositionCorrection;
            [JsonProperty(PropertyName = "maxInstructionsPerCycle")]
            public int maxInstructionsPerCycle;
            [JsonProperty(PropertyName = "gridSize")]
            public float gridSize;
            [JsonProperty(PropertyName = "droneCPUFreq")]
            public float droneCPUFreq;
        }

        protected override void LoadDefaultMessages() {
            lang.RegisterMessages(new Dictionary<string, string> {
                ["must_run_as_player"] = "this command must be run as a player",
                //["need_calibrate_perm"] = "error: you need the {0} permission to use this command",
                ["grid_pos_corr_saved"] = "grid position correction saved",
                ["calibrate_invalid_arg"] = "invalid argument format <grid_col><grid_row> ex: J13",
                ["var_already_declared"] = "line {0}: variable already declared ({1})",
                ["invalid_label"] = "line {0}: invalid label ({1})",
                ["var_not_declared"] = "line {0}: variable has not been declared ({1})",
                ["incompat_arg_type"] = "line {0}: incompatible argument type ({1}:{2}) for instruction ({3})",
                ["invalid_arg"] = "line {0}: invalid argument ({1}) for ({2}) spec: {2} {3}",
                ["invalid_instruction"] = "line {0}: invalid instruction ({1})",
                ["wrong_num_args"] = "line {0}: wrong number of arguments for ({1}) spec: {1} {2}",
            }, this);
        }

        protected override void LoadDefaultConfig() {
            var config = new ConfigData {
                maxProgramInstructions = 512,
                gridPositionCorrection = new Vector3(1, 0, 75),
                maxInstructionsPerCycle = 32,
                gridSize = 146.33f,
                droneCPUFreq = 10
            };

            Config.WriteObject(config, true);
        }

        void Init() {
            config = Config.ReadObject<ConfigData>();
            plugin = this;
            compiler = new Compiler();
            AddCovalenceCommand("minicopterdrone.calibrate", nameof(Calibrate), calibratePerm);
        }

        void Calibrate(IPlayer player, string command, string[] args) {
            if(player.Object == null) {
                Puts(lang.GetMessage("must_run_as_player", this));
                return;
            }

            Vector3 pos;
            if(args.Length == 1 && TryMapGridToPosition(args[0], out pos, false)) {
                var basePlayer = player.Object as BasePlayer;
                var actualPos = basePlayer.transform.position;
                config.gridPositionCorrection = new Vector3(actualPos.x, 0, actualPos.z) - pos;
                Config.WriteObject(config, true);
                player.Reply(lang.GetMessage("grid_pos_corr_saved", this, player.Id));
            } else {
                player.Reply(lang.GetMessage("calibrate_invalid_arg", this, player.Id));
            }
        }

        static void Print(string str) {
            Interface.Oxide.LogInfo(str, new object[]{});
        }

        class DroneManager : MonoBehaviour {
            public const string Guid = "918729b6-ca44-46c6-8ad6-1722abff10d4";
            Dictionary<uint, Drone> drones = new Dictionary<uint, Drone>();

            public static HashSet<int> activeFrequencies = new HashSet<int>();
            public static HashSet<int> risingEdgeFrequencies = new HashSet<int>();
            public static HashSet<int> fallingEdgeFrequencies = new HashSet<int>();
            List<int> removeActiveList = new List<int>();
            List<uint> removeDroneList = new List<uint>();

            public void Init() {
                float period = 1.0f / config.droneCPUFreq;
                InvokeRepeating(nameof(CycleDroneCPUs), period, period);
            }

            void CycleDroneCPUs() {
                if(drones.Count == 0) {
                    return;
                }

                risingEdgeFrequencies.Clear();
                fallingEdgeFrequencies.Clear();
                removeActiveList.Clear();
                removeDroneList.Clear();

                foreach(var freq in RFManager._broadcasters) {
                    if(freq.Value.Count > 0) {
                        if(!activeFrequencies.Contains(freq.Key)) {
                            risingEdgeFrequencies.Add(freq.Key);
                            activeFrequencies.Add(freq.Key);
                        }
                    }
                }

                foreach(var freq in activeFrequencies) {
                    List<IRFObject> list;

                    if(!RFManager._broadcasters.TryGetValue(freq, out list)) {
                        removeActiveList.Add(freq);
                        fallingEdgeFrequencies.Add(freq);
                    } else {
                        if(list.Count == 0) {
                            removeActiveList.Add(freq);
                            fallingEdgeFrequencies.Add(freq);
                        }
                    }
                }

                foreach(var freq in removeActiveList) {
                    activeFrequencies.Remove(freq);
                }

                foreach(var value in drones) {
                    var drone = value.Value;
                    
                    if(!drone.miniCopterRef.Get(true)) {
                        removeDroneList.Add(value.Key);
                        continue;
                    }

                    if(drone.active) {
                        drone.CycleCPU();
                    }
                }

                foreach(var id in removeDroneList) {
                    drones.Remove(id);
                }
            }

            void FixedUpdate() {
                foreach(var value in drones) {
                    var drone = value.Value;
                    
                    if(!drone.miniCopterRef.Get(true)) {
                        continue;
                    }

                    if(drone.active) {
                        drone.FixedUpdate();
                    }
                }
            }

            public Drone AddDrone(MiniCopter miniCopter, StorageContainer storage) {
                var drone = new Drone();
                drone.miniCopterRef.Set(miniCopter);
                drone.instanceId = miniCopter.net.ID;
                drone.storage = storage;
                drone.manager = this;
                drones.Add(drone.instanceId, drone);
                return drone;
            }

            public void OnItemAddedOrRemoved(MiniCopter miniCopter, StorageContainer storage, Item item, bool added) {
                Drone drone = null;

                if(!drones.TryGetValue(miniCopter.net.ID, out drone)) {
                    drone = AddDrone(miniCopter, storage);
                }

                drone.OnItemAddedOrRemoved(storage, item, added);
            }

            void Reset() {
                foreach(var drone in drones) {
                    drone.Value.Reset();
                }
            }
        }

        class DroneCPU {
            List<Compiler.Instruction> instructions = new List<Compiler.Instruction>();
            public Dictionary<string, int> isrs = new Dictionary<string, int>();
            public Queue<string> interrupts = new Queue<string>();
            public List<int> picStack = new List<int>();
            int pic = 0;
            bool abort = false;
            string abortReason = null;
            Dictionary<string, Compiler.Instruction.Argument> numVariables = new Dictionary<string, Compiler.Instruction.Argument>();
            List<float> stack = new List<float>();
            public int maxStackSize = 64;

            public void Reset() {
                pic = 0;
                interrupts.Clear();
                picStack.Clear();
                numVariables.Clear();
            }

            public void LoadInstructions(List<Compiler.Instruction> instructions) {
                this.instructions = instructions;
                isrs.Clear();
                interrupts.Clear();
                picStack.Clear();
                numVariables.Clear();
                stack.Clear();
                pic = 0;

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "isr") {
                        isrs.Add(instr.args[0].stringValue, i);
                    } else if(instr.name == "num") {
                        numVariables.Add(instr.args[0].stringValue, instr.args[0]);
                    }
                }
            }

            public void Jump(int addr) {
                pic = addr;
            }

            public void Call(int addr) {
                picStack.Add(pic);
                pic = addr;
            }

            public bool Ret() {
                if(picStack.Count == 0) {
                    return false;
                }

                Jump(picStack[picStack.Count - 1]);
                picStack.RemoveAt(picStack.Count - 1);

                return true;
            }

            public void Interrupt(string name) {
                if(isrs.ContainsKey(name)) {
                    interrupts.Enqueue(name);
                }
            }

            public bool WriteVariable(string name, float value) {
                Compiler.Instruction.Argument arg;
                if(numVariables.TryGetValue(name, out arg)) {
                    arg.floatValue = value;
                    arg.intValue = (int)value;
                    return true;
                }

                return false;
            }

            public bool HandlePendingInterrupt() {
                if(interrupts.Count == 0) {
                    return false;
                }

                var isr = interrupts.Dequeue();
                int addr;
                isrs.TryGetValue(isr, out addr);
                Call(addr);

                return true;
            }

            public void Abort(string reason) {
                abort = true;
                abortReason = reason;
            }

            public bool Cycle(out Compiler.Instruction instr, out string failReason) {
                if(instructions == null || pic < 0 || pic >= instructions.Count) {
                    instr = null;
                    failReason = "out of instructions";
                    return false;
                }

                if(abort) {
                    instr = null;
                    failReason = abortReason;
                    return false;
                }

                instr = instructions[pic++];

                // if maybe one arg instruction
                if(instr.args.Count == 1) {
                    bool isOneOfThese = true;

                    switch(instr.name) {
                        case "jmp":
                            Jump(instr.args[0].intValue);
                            break;
                        case "call":
                            Call(instr.args[0].intValue);
                            break;
                        case "int":
                            Interrupt(instr.args[0].stringValue);
                            break;
                        case "ret":
                            if(!Ret()) {
                                failReason = "no address to return from";
                                return false;
                            }

                            break;
                        case "push":
                            if(stack.Count == maxStackSize) {
                                failReason = $"maximum stack size exceeded ({maxStackSize})";
                                return false;
                            }

                            stack.Add(instr.args[0].floatValue);
                            break;
                        default:
                            isOneOfThese = false;
                            break;
                    }

                    if(isOneOfThese) {
                        failReason = null;
                        return true;
                    }
                }

                // if maybe instruction that assigns to variable as first arg
                if(instr.args.Count > 0 && instr.args[0].paramType == Compiler.ParamType.NumVariable) {
                    bool isOneOfThese = true;
                    float result = 0;

                    switch(instr.name) {
                        case "pop":
                            if(stack.Count == 0) {
                                failReason = "stack empty";
                                return false;
                            }

                            result = stack[stack.Count - 1];
                            stack.RemoveAt(stack.Count - 1);
                            break;
                        case "mov":
                            result = instr.args[1].floatValue;
                            break;
                        default:
                            isOneOfThese = false;
                            break;
                    }

                    if(isOneOfThese) {
                        instr.args[0].floatValue = result;
                        instr.args[0].intValue = (int)result;
                        failReason = null;
                        return true;
                    }
                }

                // if might be one arg math instruction
                if(instr.args.Count == 1 && instr.args[0].paramType == Compiler.ParamType.Num) {
                    var valueArg = instr.args[0];
                    float result = 0;
                    bool isOneOfThese = true;
                    
                    try {
                        switch(instr.name) {
                            case "abs":
                                result = Mathf.Abs(valueArg.floatValue);
                                break;
                            case "sign":
                                result = Mathf.Sign(valueArg.floatValue);
                                break;
                            case "sqrt":
                                result = Mathf.Sqrt(valueArg.floatValue);
                                break;
                            case "round":
                                result = Mathf.Round(valueArg.floatValue);
                                break;
                            case "floor":
                                result = Mathf.Floor(valueArg.floatValue);
                                break;
                            case "ceil":
                                result = Mathf.Ceil(valueArg.floatValue);
                                break;
                            default:
                                isOneOfThese = false;
                                break;
                        }

                        if(isOneOfThese) {
                            if(!float.IsFinite(result)) {
                                failReason = $"math error: {instr.name} {String.Join(" ", instr.args.Select(x => x.rawValue))} => {result}";
                                return false;
                            }

                            WriteVariable("rslt", result);
                            failReason = null;
                            return true;
                        }
                    } catch(ArithmeticException e) {
                        failReason = e.ToString();
                        return false;
                    }
                }

                // if might be two arg math instruction
                if(instr.args.Count == 2 && instr.args[0].paramType == Compiler.ParamType.Num && instr.args[1].paramType == Compiler.ParamType.Num) {
                    var lhsArg = instr.args[0];
                    var rhsArg = instr.args[1];
                    float result = 0;
                    bool isOneOfThese = true;
                    
                    try {
                        switch(instr.name) {
                            case "add":
                                result = lhsArg.floatValue + rhsArg.floatValue;
                                break;
                            case "sub":
                                result = lhsArg.floatValue - rhsArg.floatValue;
                                break;
                            case "mul":
                                result = lhsArg.floatValue * rhsArg.floatValue;
                                break;
                            case "div":
                                result = lhsArg.floatValue / rhsArg.floatValue;
                                break;
                            case "pow":
                                result = Mathf.Pow(lhsArg.floatValue, rhsArg.floatValue);
                                break;
                            case "min":
                                result = Mathf.Min(lhsArg.floatValue, rhsArg.floatValue);
                                break;
                            case "max":
                                result = Mathf.Max(lhsArg.floatValue, rhsArg.floatValue);
                                break;
                            default:
                                isOneOfThese = false;
                                break;
                        }

                        if(isOneOfThese) {
                            if(!float.IsFinite(result)) {
                                failReason = $"math error: {instr.name} {String.Join(" ", instr.args.Select(x => x.rawValue))} => {result}";
                                return false;
                            }

                            WriteVariable("rslt", result);
                            failReason = null;
                            return true;
                        }
                    } catch(ArithmeticException e) {
                        failReason = e.ToString();
                        return false;
                    }
                }

                if(instr.name == "lerp") {
                    float result = Mathf.Lerp(instr.args[0].floatValue, instr.args[1].floatValue, instr.args[2].floatValue);
                    WriteVariable("rslt", result);
                    failReason = null;
                    return true;
                }

                // if might be conditional jump instruction
                if(instr.args.Count == 3 && instr.args[0].paramType == Compiler.ParamType.Num && instr.args[1].paramType == Compiler.ParamType.Num && instr.args[2].paramType == Compiler.ParamType.Address) {
                    float lhs = instr.args[0].floatValue;
                    float rhs = instr.args[1].floatValue;
                    int addr = instr.args[2].intValue;
                    bool jump = false;

                    switch(instr.name) {
                        case "ja":
                            jump = Mathf.Approximately(lhs, rhs);
                            break;
                        case "je":
                            jump = lhs == rhs;
                            break;
                        case "jne":
                            jump = lhs != rhs;
                            break;
                        case "jna":
                            jump = !Mathf.Approximately(lhs, rhs);
                            break;
                        case "jg":
                            jump = lhs > rhs;
                            break;
                        case "jge":
                            jump = lhs >= rhs;
                            break;
                        case "jl":
                            jump = lhs < rhs;
                            break;
                        case "jle":
                            jump = lhs <= rhs;
                            break;
                    }

                    if(jump) {
                        Jump(addr);
                        failReason = null;
                        return true;
                    }
                }

                failReason = null;
                return true;
            }
        }

        class Drone {
            public DroneManager manager;
            public EntityRef miniCopterRef;
            public uint instanceId;
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

            public void OnItemAddedOrRemoved(StorageContainer storage, Item item, bool added) {
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

                if(!HasFlag(Flag.EngineOn) && !HasFlag(Flag.EngineStarting) && !copter.Waterlogged()) {
                    engineStartTime = Time.fixedTime;
                    SetFlag(Flag.EngineStarting, true);
                    copter.EngineStartup();
                }
            }

            void StopEngine() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                SetFlag(Flag.EngineOn, false);
                SetFlag(Flag.EngineStarting, false);
                copter.EngineOff();
            }

            void UpdateEngine() {
                var copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter.HasFuel()) {
                    StopEngine();
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
            }

            public void CycleCPU() {
                MiniCopter copter = miniCopterRef.Get(true) as MiniCopter;

                if(!copter) {
                    Reset();
                    return;
                }

                var currentAltitude = GetAltitude(copter.transform.position);

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
                while(numInstructionsExecuted < config.maxInstructionsPerCycle && !HasFlag(Flag.ExecutingInstruction)) {
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

        static bool TryMapGridToPosition(string coord, out Vector3 result, bool useCorrection = true) {
             float mapSize = ConVar.Server.worldsize;
            
            if(coord.Length <= 4) {
                var match = Regex.Match(coord, @"^([A-Za-z]{1,2})([0-9]{1,2})$");

                if(match.Success && match.Groups.Count == 3) {
                    var groups = match.Groups;
                    string letter = groups[1].ToString().ToUpper();
                    string numberWholeStr = groups[2].ToString();

                    int letterNumber = (letter.Length == 1) ? letter[0] - 'A' : letter[1] + 26 - 'A';
                    float row;
                    float.TryParse(numberWholeStr, out row);

                    float x = letterNumber * config.gridSize - (mapSize / 2);
                    float y = -row * config.gridSize + (mapSize / 2);

                    result = new Vector3(x, 0, y);

                    if(useCorrection) {
                        result += config.gridPositionCorrection;
                    }

                    return true;
                }
            }

            result = new Vector2();
            return false;
        }

        static Vector3 MapGridToPosition(float col, float row, bool useCorrection = true) {
            float mapSize = ConVar.Server.worldsize;
            float x = col * config.gridSize - (mapSize / 2);
            float y = -row * config.gridSize + (mapSize / 2);

            var result = new Vector3(x, 0, y);

            if(useCorrection) {
                result += config.gridPositionCorrection;
            }

            return result;
        }

        void OnEntitySpawned(BaseNetworkable entity)
        {
            var miniCopter = entity as MiniCopter;

            if(miniCopter) {
                if(miniCopter.ShortPrefabName != "minicopter.entity") {
                    return;
                }

                ProcessMiniCopter(miniCopter, null);
            }
        }

        class Compiler {
            List<string[]> tokens = new List<string[]>();
            public List<string> errors = new List<string>();
            public List<Instruction> instructions = new List<Instruction>();

            public enum ParamType {
                Num,
                Identifier,
                Address,
                NumVariable
            }

            public class Instruction {
                public class Argument {
                    public string name;
                    public ParamType paramType;
                    public ParamType argType;
                    public string rawValue;
                    public string stringValue;
                    public Argument variableReference = null;

                    private int _intValue;
                    public int intValue {
                        get {
                            if(variableReference == null) {
                                return _intValue;
                            } else {
                                return variableReference._intValue;
                            }
                        } set {
                            if(variableReference == null) {
                                _intValue = value;
                            } else {
                                variableReference._intValue = value;
                            }
                        }
                    }

                    private float _floatValue;
                    public float floatValue {
                        get {
                            if(variableReference == null) {
                                return _floatValue;
                            } else {
                                return variableReference._floatValue;
                            }
                        } set {
                            if(variableReference == null) {
                                _floatValue = value;
                            } else {
                                variableReference._floatValue = value;
                            }
                        }
                    }
                }

                public string name;
                public List<Argument> args;

                public Instruction(string name) {
                    this.name = name;
                    this.args = new List<Argument>();
                }

                public override string ToString() {
                    return $"{name} {String.Join(" ", args.Select(x => $"<{x.name}:{x.paramType}>"))}";
                }
            }

            struct Param {
                public ParamType type;
                public string name;

                public Param(string name, ParamType type) {
                    this.name = name;
                    this.type = type;
                }
            }

            Dictionary<string, Param[]> instructionDefs = new Dictionary<string, Param[]> {
                {"label", new Param[] { new Param("name", ParamType.Identifier) }},
                {"isr", new Param[] { new Param("name", ParamType.Identifier) }},
                {"jmp", new Param[] { new Param("label_name", ParamType.Address) }},
                {"call", new Param[] { new Param("label_name", ParamType.Address) }},
                {"int", new Param[] { new Param("isr_name", ParamType.Identifier) }},
                {"ret", new Param[] {  }},
                {"nop", new Param[] {  }},
                {"num", new Param[] { new Param("var_name_decl", ParamType.Identifier) }},
                {"push", new Param[] { new Param("value", ParamType.Num) }},

                {"pop", new Param[] { new Param("var_name", ParamType.NumVariable) }},
                {"mov", new Param[] { new Param("dest/lhs", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},

                {"lerp", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("t", ParamType.Num) }},

                {"add", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"sub", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"mul", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"div", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"pow", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"min", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},
                {"max", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num) }},

                {"abs", new Param[] { new Param("value", ParamType.Num) }},
                {"sign", new Param[] { new Param("value", ParamType.Num) }},
                {"sqrt", new Param[] { new Param("value", ParamType.Num) }},
                {"floor", new Param[] { new Param("value", ParamType.Num) }},
                {"ceil", new Param[] { new Param("value", ParamType.Num) }},
                {"round", new Param[] { new Param("value", ParamType.Num) }},

                {"ja", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"je", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jne", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jna", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jg", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jge", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jl", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jle", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},

                {"startengine", new Param[] {  }},
                {"stopengine", new Param[] {  }},
                {"land", new Param[] { new Param("speed", ParamType.Num) }},
                {"sleep", new Param[] { new Param("seconds", ParamType.Num) }},
                {"target", new Param[] { new Param("grid_col", ParamType.Num), new Param("grid_row", ParamType.Num), new Param("altitude", ParamType.Num) }},
                {"targetalt", new Param[] { new Param("altitude", ParamType.Num) }},
                {"targethere", new Param[] {  }},
                {"targetrf", new Param[] { new Param("frequency", ParamType.Num) }},
                {"flyto", new Param[] { }},
                {"flythrough", new Param[] { }},
                {"drop", new Param[] { new Param("slot", ParamType.Num) }},
                {"waitrf", new Param[] { new Param("frequency", ParamType.Num) }},

                {"pushtarget", new Param[] {  }},
                {"poptarget", new Param[] {  }},
                {"pushtargetalt", new Param[] {  }},
                {"poptargetalt", new Param[] {  }},

                {"fly", new Param[] {  }},
                {"descend", new Param[] { new Param("speed", ParamType.Num) }},
                {"gettarget", new Param[] { new Param("stack_pos", ParamType.Num), new Param("x", ParamType.NumVariable), new Param("y", ParamType.NumVariable), new Param("z", ParamType.NumVariable) }},
                {"settarget", new Param[] { new Param("stack_pos", ParamType.Num), new Param("x", ParamType.Num), new Param("y", ParamType.Num), new Param("z", ParamType.Num) }},
                {"getalt", new Param[] { new Param("stack_pos", ParamType.Num), new Param("altitude", ParamType.NumVariable) }},
                {"setalt", new Param[] { new Param("stack_pos", ParamType.Num), new Param("altitude", ParamType.Num) }},
                {"setpitch", new Param[] { new Param("pitch", ParamType.Num) }},
            };

            public bool Compile(string code) {
                errors.Clear();
                code = Regex.Replace(code, @"#.*$", "", RegexOptions.Multiline);

                Tokenize(code);

                // registers r0 - r7
                for(int i = 0; i < 8; i++) {
                    tokens.Add(new string[]{"num", "r" + i});
                }

                // rslt register
                tokens.Add(new string[]{"num", "rslt"});

                if(!Parse()) {
                    return false;
                }

                var labelAddresses = new Dictionary<string, int>();
                var variables = new Dictionary<string, Instruction.Argument>();

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "label") {
                        labelAddresses.Add(instr.args[0].stringValue, i);
                    }

                    if(instr.name == "num") {
                        var arg = instr.args[0];

                        if(variables.ContainsKey(arg.stringValue)) {
                            var message = plugin.lang.GetMessage("var_already_declared", plugin);
                            errors.Add(string.Format(message, i, arg.stringValue));
                            return false;
                        }

                        variables.Add(arg.stringValue, arg);
                    }
                }

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    for(int j = 0; j < instr.args.Count; j++) {
                        var arg = instr.args[j];

                        if(arg.paramType == ParamType.Address) {
                            int addr;
        
                            if(!labelAddresses.TryGetValue(arg.stringValue, out addr)) {
                                var message = plugin.lang.GetMessage("invalid_label", plugin);
                                errors.Add(string.Format(message, i, arg.stringValue));
                                return false;
                            }

                            arg.intValue = addr;
                        }

                        if(arg.argType == ParamType.NumVariable) {
                            Instruction.Argument variableArg;
                            if(!variables.TryGetValue(arg.stringValue, out variableArg)) {
                                var message = plugin.lang.GetMessage("var_not_declared", plugin);
                                errors.Add(string.Format(message, i, arg.stringValue));
                                return false;
                            } else {
                                arg.variableReference = variableArg;
                            }
                        }

                        if(arg.paramType == ParamType.Num) {
                            if(arg.argType != ParamType.Num && arg.argType != ParamType.NumVariable) {
                                var message = plugin.lang.GetMessage("incompat_arg_type", plugin);
                                errors.Add(string.Format(message, i, arg.stringValue, arg.argType, instr.ToString()));
                                return false;
                            }
                        }
                    }
                }

                return true;
            }

            bool ProcessInstruction(int line, string instr, string[] args, Param[] parameters) {
                var compiledInstruction = new Instruction(instr);

                Action<string> fail = (string arg) => {
                    var message = plugin.lang.GetMessage("invalid_arg", plugin);
                    errors.Add(string.Format(message, line, arg, instr, String.Join(" ", parameters.Select(x => x.name + ':' + x.type))));
                };

                Action<int, float, int, string, ParamType> addArgument = (int index, float floatValue, int intValue, string stringValue, Compiler.ParamType argType) => {
                    compiledInstruction.args.Add(new Instruction.Argument {
                        name = parameters[index].name,
                        paramType = parameters[index].type,
                        argType = argType,
                        rawValue = args[index],
                        floatValue = floatValue,
                        intValue = intValue,
                        stringValue = stringValue
                    });
                };

                for(int i = 0; i < args.Length; i++) {
                    var arg = args[i];
                    var param = parameters[i];

                    if(param.type == ParamType.Num) {
                        var match = Regex.Match(arg, @"^[+-]*[0-9]*[\.]?[0-9]*$");
                        var matchMapGridCol = Regex.Match(arg, @"^([a-zA-Z]{1,2})[\.]([0-9]*)?$");
                        var matchNumVar = Regex.Match(arg, @"^[a-zA-Z_][a-zA-Z0-9_]*$");

                        if(!match.Success && !matchMapGridCol.Success && !matchNumVar.Success) {
                            fail(arg);
                            return false;
                        }

                        if(match.Success) {
                            if(match.Groups[0].ToString() == ".") {
                                fail(arg);
                                return false;
                            }

                            float value;
                            if(!float.TryParse(arg, out value)) {
                                fail(arg);
                                return false;
                            }

                            addArgument(i, value, (int)value, arg, param.type);
                        } else if(matchMapGridCol.Success) {
                            var lettersStr = matchMapGridCol.Groups[1].ToString().ToUpper();
                            var lettersFractionStr = matchMapGridCol.Groups[2].ToString();

                            int lettersWhole = (lettersStr.Length == 1) ? lettersStr[0] - 'A' : lettersStr[1] + 26 - 'A';
                            float lettersFraction = 0.0f;

                            if(lettersFractionStr != "." && lettersFractionStr.Length != 0) {
                                if(!float.TryParse(lettersFractionStr, out lettersFraction)) {
                                    fail(arg);
                                    return false;
                                }
                            }

                            var value = lettersWhole + lettersFraction;
                            
                            addArgument(i, value, (int)value, arg, param.type);
                        } else if(matchNumVar.Success) {
                            addArgument(i, 0, 0, arg, ParamType.NumVariable);
                        }
                    }

                    if(param.type == ParamType.Identifier || param.type == ParamType.Address || param.type == ParamType.NumVariable) {
                        var match = Regex.Match(arg, @"^[a-zA-Z_][a-zA-Z0-9_]*$");

                        if(!match.Success) {
                            fail(arg);
                            return false;
                        }
                        
                        addArgument(i, 0, 0, arg, param.type);
                    }
                }

                instructions.Add(compiledInstruction);
                return true;
            }

            bool Parse() {
                instructions.Clear();

                for(int i = 0; i < tokens.Count; i++) {
                    var line = tokens[i];
                    var instr = line[0];

                    Param[] parameters;
                    var found = instructionDefs.TryGetValue(instr, out parameters);

                    if(!found) {
                        var message = plugin.lang.GetMessage("invalid_instruction", plugin);
                        errors.Add(string.Format(message, i, instr));
                        return false;
                    }

                    if(parameters.Length != line.Length - 1) {
                        var message = plugin.lang.GetMessage("wrong_num_args", plugin);
                        errors.Add(string.Format(message, i, instr, String.Join(" ", parameters.Select(x => x.name + ':' + x.type))));
                        return false;
                    }

                    if(!ProcessInstruction(i, instr, line.Skip(1).ToArray(), parameters)) {
                        return false;
                    }
                }

                return true;
            }

            void Tokenize(string code) {
                tokens.Clear();
                var lines = code.Split(new[] {"\r\n", "\r", "\n", ";"}, StringSplitOptions.RemoveEmptyEntries);

                for(int i = 0; i < lines.Length; i++) {
                    lines[i] = code = Regex.Replace(lines[i].Trim() ,@"\s+"," ");

                    if(lines[i].Length > 0) {
                        if(lines[i][0] == '#') {
                            continue;
                        }

                        var args = lines[i].Split(' ');

                        if(args.Length > 0) {
                            tokens.Add(args);
                        }
                    }
                }
            }
        }

        StorageContainer ProcessMiniCopter(MiniCopter miniCopter, StorageContainer existingStorage) {
            StorageContainer storage;

            if(existingStorage == null) {
                var ent = GameManager.server.CreateEntity("assets/prefabs/deployable/woodenbox/woodbox_deployed.prefab");

                var collider = ent.GetComponent<Collider>();
                
                if(collider) {
                    GameObject.Destroy(collider);
                }
            
                ent.name = "minicopterdrone.storage";
                ent.Spawn();
                ent.transform.localPosition = new Vector3(0, 0.2f, -1.2f);
                ent.transform.localRotation = Quaternion.identity;
                ent.SetParent(miniCopter, false, false);

                storage = ent.GetComponent<StorageContainer>();
            } else {
                storage = existingStorage;
            }

            storage.inventory.onItemAddedRemoved = new Action<Item, bool>((Item item, bool added) => {
                droneManager.OnItemAddedOrRemoved(miniCopter, storage, item, added);
                storage.OnItemAddedOrRemoved(item, added);
            });

            return storage;
        }

        void OnServerInitialized() {
            var go = new GameObject(DroneManager.Guid);
            droneManager = go.AddComponent<DroneManager>();
            droneManager.Init();

            foreach(var miniCopter in GameObject.FindObjectsOfType<MiniCopter>()) {
                StorageContainer storage = null;

                // scraptransportheli dervies from MiniCopter
                if(miniCopter.ShortPrefabName != "minicopter.entity") {
                    continue;
                }

                foreach(var ent in miniCopter.children) {
                    var container = ent as StorageContainer;

                    if(container) {
                        if(container.name == "minicopterdrone.storage") {
                            storage = container;
                        }
                    }
                }

                ProcessMiniCopter(miniCopter, storage);
            }
        }

        void Unload() {
            if(droneManager) {
                GameObject.Destroy(droneManager);
            }

            plugin = null;
            config = null;
            compiler = null;
        }

        static class PIDController {
            public class Base {
                public float p;
                public float i;
                public float d;
                public float spRC;
                public float dRC;
                public float outputRC;
                public bool reset = true;

                public void Reset() {
                    reset = true;
                }
            }

            public class Angular : Base {
                Vector3 errorLast;
                Vector3 errorAccum = Vector3.zero;
                Vector3 filteredSp = Vector3.zero;
                Vector3 filteredD = Vector3.zero;
                Vector3 filteredOutput = Vector3.zero;

                public Vector3 Update(Vector3 sp, Vector3 pv, float dt) {
                    if(reset) {
                        filteredSp = sp;
                    }

                    filteredSp = Vector3.Lerp(filteredSp, sp, dt / (spRC + dt));
                    var cross = Vector3.Cross(pv, sp);

                    if(cross.magnitude < 1e-6f) {
                        return Vector3.zero;
                    }
                    
                    var dot = Vector3.Dot(sp, pv);
                    var error = cross.normalized * Mathf.Acos(Mathf.Clamp(dot, -1f, 1f));

                    if(reset) {
                        errorLast = error;
                    }

                    errorAccum += error * dt;
                    var dTerm = (error - errorLast) / dt;

                    if(reset) {
                       filteredD = dTerm;
                    }

                    filteredD = Vector3.Lerp(filteredD, dTerm, dt / (dRC + dt));
                    var result = (error * p) + (errorAccum * i) + (filteredD * d);

                    if(reset) {
                        filteredOutput = result;
                        reset = false;
                    }

                    filteredOutput = Vector3.Lerp(filteredOutput, result, dt / (outputRC + dt));
                    errorLast = error;
                    return filteredOutput;
                }
            }

            public class Linear : Base {
                float errorLast;
                float errorAccum = 0;
                float filteredSp = 0;
                float filteredD = 0;
                float filteredOutput = 0;

                public float Update(float sp, float pv, float dt) {
                    if(reset) {
                        filteredSp = sp;
                    }

                    filteredSp = Mathf.Lerp(filteredSp, sp, dt / (spRC + dt));
                    var error = filteredSp - pv;

                    if(reset) {
                        errorLast = error;
                    }

                    errorAccum += error * dt;
                    var dTerm = (error - errorLast) / dt;

                    if(reset) {
                       filteredD = dTerm;
                    }

                    filteredD = Mathf.Lerp(filteredD, dTerm, dt / (dRC + dt));
                    var result = (error * p) + (errorAccum * i) + (filteredD * d);

                    if(reset) {
                        filteredOutput = result;
                        reset = false;
                    }

                    filteredOutput = Mathf.Lerp(filteredOutput, result, dt / (outputRC + dt));
                    errorLast = error;
                    return filteredOutput;
                }
            }

            public class Vector : Base {
                Vector3 errorLast;
                Vector3 errorAccum = Vector3.zero;
                Vector3 filteredD = Vector3.zero;
                Vector3 filteredSp = Vector3.zero;
                Vector3 filteredOutput = Vector3.zero;

                public Vector3 Update(Vector3 sp, Vector3 pv, float dt) {
                    if(reset) {
                        filteredSp = sp;
                    }

                    filteredSp = Vector3.Lerp(filteredSp, sp, dt / (spRC + dt));
                    var error = filteredSp - pv;

                    if(reset) {
                        errorLast = error;
                    }

                    errorAccum += error * dt;
                    var dTerm = (error - errorLast) / dt;

                    if(reset) {
                       filteredD = dTerm;
                    }

                    filteredD = Vector3.Lerp(filteredD, dTerm, dt / (dRC + dt));
                    var result = (error * p) + (errorAccum * i) + (filteredD * d);

                    if(reset) {
                        filteredOutput = result;
                        reset = false;
                    }

                    filteredOutput = Vector3.Lerp(filteredOutput, result, dt / (outputRC + dt));
                    errorLast = error;
                    return filteredOutput;
                }
            }
        }
    }
}