using System;
using Oxide.Core;
using Oxide.Core.Configuration;
using Oxide.Core.Plugins;
using Oxide.Game.Rust.Cui;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Text.RegularExpressions;

using UnityEngine;
using UnityEngine.Events;

namespace Oxide.Plugins
{
    [Info("MiniCopterDrone", "Andrew", "1.0.0")]
    public class MiniCopterDrone : RustPlugin
    {
        static MiniCopterDrone plugin;
        static public BasePlayer debugPlayer = null;
        DroneManager droneManager = null;
        static ConfigData config;
        static Compiler compiler = new Compiler();

        class ConfigData {
            [JsonProperty(PropertyName = "maxProgramInstructions")]
            public int maxProgramInstructions;
            [JsonProperty(PropertyName = "gridPositionCorrection")]
            public Vector3 gridPositionCorrection;
            [JsonProperty(PropertyName = "debugPlayerId")]
            public ulong debugPlayerId;
        }

        protected override void LoadDefaultConfig() {
            var config = new ConfigData {
                maxProgramInstructions = 128,
                gridPositionCorrection = new Vector3(0.44f, 0, -74.47f)
            };

            Config.WriteObject(config, true);
        }

        void Init() {
            config = Config.ReadObject<ConfigData>();
        }

        [ConsoleCommand("minicopterdrone.calibrate")]
        void Calibrate(ConsoleSystem.Arg argument) {
            Vector3 pos;

            if( TryMapGridToPosition(argument.Args[0], out pos, false)) {
                var actualPos = argument.Player().transform.position;
                config.gridPositionCorrection = new Vector3(actualPos.x, 0, actualPos.z) - pos;
                Config.WriteObject(config, true);
                argument.ReplyWith($"gridPositionCorrection: {config.gridPositionCorrection.ToString()}");
            } else {
                argument.ReplyWith("failed");
            }
        }

        static void Print(string str) {
            Interface.Oxide.LogInfo(str, new object[]{});
        }

        class DroneManager : MonoBehaviour {
            public const string Guid = "918729b6-ca44-46c6-8ad6-1722abff10d4";
            Dictionary<int, Drone> drones = new Dictionary<int, Drone>();
            public MiniCopterDrone plugin = null;

            public static HashSet<int> activeFrequencies = new HashSet<int>();
            public static HashSet<int> risingEdgeFrequencies = new HashSet<int>();
            public static HashSet<int> fallingEdgeFrequencies = new HashSet<int>();

            void FixedUpdate() {
                risingEdgeFrequencies.Clear();
                fallingEdgeFrequencies.Clear();

                foreach(var freq in RFManager._broadcasters) {
                    if(freq.Value.Count > 0) {
                        if(!activeFrequencies.Contains(freq.Key)) {
                            risingEdgeFrequencies.Add(freq.Key);
                            activeFrequencies.Add(freq.Key);
                        }
                    }
                }

                foreach(var freq in activeFrequencies.ToArray()) {
                    List<IRFObject> list;

                    if(!RFManager._broadcasters.TryGetValue(freq, out list)) {
                        activeFrequencies.Remove(freq);
                        fallingEdgeFrequencies.Add(freq);
                    } else {
                        if(list.Count == 0) {
                            activeFrequencies.Remove(freq);
                            fallingEdgeFrequencies.Add(freq);
                        }
                    }
                }

                foreach(var value in drones) {
                    var drone = value.Value;
                    
                    if(!drone.miniCopterRef.Get(true)) {
                        drones.Remove(value.Key);
                        return;
                    }

                    drone.FixedUpdate();
                }
            }

            public bool OnItemAddedOrRemoved(MiniCopter miniCopter, StorageContainer storage, Item item, bool added) {
                Drone drone = null;

                if(!drones.TryGetValue(miniCopter.GetInstanceID(), out drone)){
                    drone = new Drone();

                    drone.miniCopterRef.Set(miniCopter);
                    drone.instanceId = miniCopter.GetInstanceID();
                    drone.storage = storage;
                    drone.manager = this;

                    drones.Add(miniCopter.GetInstanceID(), drone);
                }

                drone.OnItemAddedOrRemoved(storage, item, added);

                return true;
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

            Dictionary<string, Compiler.Instruction> numVariables = new Dictionary<string, Compiler.Instruction>();

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
                pic = 0;

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "isr") {
                        isrs.Add(instr.args[0].stringValue, i);
                    } else if(instr.name == "num") {
                        numVariables.Add(instr.args[0].stringValue, instr);
                    }
                }

                for(int i = 0; i < 8; i++) {
                    var instruction = new Compiler.Instruction("r" + i);

                    instruction.args.Add(new Compiler.Instruction.Argument {
                        name = null,
                        paramType = Compiler.ParamType.NumVariable,
                        argType = Compiler.ParamType.NumVariable,
                        rawValue = null,
                        addressValue = 0,
                        floatValue = 0,
                        intValue = 0,
                        stringValue = null
                    });

                    numVariables.Add(instruction.name, instruction);
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
                Compiler.Instruction instr;
                if(numVariables.TryGetValue(name, out instr)) {
                    instr.args[0].floatValue = value;
                    instr.args[0].intValue = (int)value;
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

                for(int i = 0; i < instr.args.Count; i++) {
                    var arg = instr.args[i];

                    if(arg.argType == Compiler.ParamType.NumVariable) {
                        Compiler.Instruction value;
                        if(numVariables.TryGetValue(arg.stringValue, out value)) {
                            arg.floatValue = value.args[0].floatValue;
                            arg.intValue = value.args[0].intValue;
                        } else {
                            failReason = $"couldn't find variable \"{arg.stringValue}\"";
                            return false;
                        }
                    }
                }

                if(instr.name == "label" || instr.name == "isr" || instr.name == "var") {
                    return Cycle(out instr, out failReason);
                }

                if(instr.name == "jmp") {
                    Jump(instr.args[0].addressValue);
                }

                if(instr.name == "call") {
                    Call(instr.args[0].addressValue);
                }

                if(instr.name == "int") {
                    Interrupt(instr.args[0].stringValue);
                }

                if(instr.name == "ret") {
                    if(!Ret()) {
                        failReason = "no address to return from";
                        return false;
                    }
                }

                if(
                    instr.name == "mov"
                    || instr.name == "add"
                    || instr.name == "sub"
                    || instr.name == "mul"
                    || instr.name == "div"
                    || instr.name == "abs"
                    || instr.name == "sqrt"
                    || instr.name == "pow"
                    ) {
                    Compiler.Instruction variable;
                    if(!this.numVariables.TryGetValue(instr.args[0].stringValue, out variable)) {
                        failReason = $"couldn't find variable \"{instr.args[0].stringValue}\"";
                        return false;
                    }

                    var variableArg = variable.args[0];
                    var valueArg = instr.args[1];
                    
                    try {
                        switch(instr.name) {
                            case "mov":
                                variableArg.floatValue = valueArg.floatValue;
                                break;
                            case "add":
                                variableArg.floatValue += valueArg.floatValue;
                                break;
                            case "sub":
                                variableArg.floatValue -= valueArg.floatValue;
                                break;
                            case "mul":
                                variableArg.floatValue *= valueArg.floatValue;
                                break;
                            case "div":
                                variableArg.floatValue /= valueArg.floatValue;
                                break;
                            case "abs":
                                variableArg.floatValue = Mathf.Abs(valueArg.floatValue);
                                break;
                            case "sqrt":
                                variableArg.floatValue = Mathf.Sqrt(valueArg.floatValue);
                                break;
                            case "pow":
                                variableArg.floatValue = Mathf.Pow(variableArg.floatValue, valueArg.floatValue);
                                break;
                        }

                        if(!float.IsFinite(variableArg.floatValue)) {
                            failReason = $"math error: {instr.name} {String.Join(" ", instr.args.Select(x => x.rawValue))} => {variableArg.floatValue}";
                            return false;
                        }
                    } catch(Exception e) {
                        failReason = e.ToString();
                        return false;
                    }

                    variableArg.intValue = (int)variableArg.floatValue;
                }

                if(
                    instr.name == "je"
                    || instr.name == "jne"
                    || instr.name == "jg"
                    || instr.name == "jge"
                    || instr.name == "jl"
                    || instr.name == "jle"
                    ) {
                    float lhs = instr.args[0].floatValue;
                    float rhs = instr.args[1].floatValue;
                    int addr = instr.args[2].addressValue;
                    bool jump = false;

                    switch(instr.name) {
                        case "je":
                            jump = lhs == rhs;
                            break;
                        case "jne":
                            jump = lhs != rhs;
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
                    }
                }

                failReason = null;
                return true;
            }
        }

        class Drone {
            public DroneManager manager;
            public EntityRef miniCopterRef;
            public int instanceId;
            DroneCPU cpu = new DroneCPU();
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
            bool haveTarget = false;
            float sleepTime = -1;
            float engineStartTime = -1;
            float landingSpeed = 0;
            float landingTime;
            float desiredPitch = 0;

            public void Reset() {
                active = false;
                currentInstruction = null;
                desiredAltitude = 0;
                target = Vector3.zero;
                haveTarget = false;
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
                    var lines = item.text.ToLower().Split(new[] {"\r\n", "\r", "\n"}, StringSplitOptions.RemoveEmptyEntries);

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

                /*if(this.currentInstruction != null) {
                    var flagValues = Enum.GetValues(typeof(Flag));
                    var flagNames = Enum.GetNames(typeof(Flag));
                    var setFlags = new List<string>();

                    for(int i = 0; i < flagValues.Length; i++) {
                        var val = (Flag)flagValues.GetValue(i);
                        if(HasFlag(val)) {
                            setFlags.Add(flagNames[i]);
                        }
                    }

                    plugin.SendReply(debugPlayer, $"{this.currentInstruction.name} {String.Join(", ", setFlags)}");
                    plugin.SendReply(debugPlayer, $"this.target: {this.target}");
                    plugin.SendReply(debugPlayer, $"this.desiredAltitude: {this.desiredAltitude}");
                    plugin.SendReply(debugPlayer, $"this.desiredPitch: {this.desiredPitch}");
                }*/

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

                        if(this.currentInstruction.name == "land") {
                            SetFlag(Flag.ExecutingInstruction, false);
                        }
                    }
                }

                if(HasFlag(Flag.HasTarget) && HasFlag(Flag.SeekingTarget)) {
                    Vector3 offset = this.target - copter.transform.position;

                    if(offset.magnitude < 1 && copter.rigidBody.velocity.magnitude < 1) {
                        SetFlag(Flag.SeekingTarget, false);
                        cpu.Interrupt("at_target");

                        if(this.currentInstruction.name == "flyto") {
                            SetFlag(Flag.ExecutingInstruction, false);
                        }
                    }
                }

                if(HasFlag(Flag.HasTarget) && HasFlag(Flag.SeekingTargetFlythrough)) {
                    Vector3 offset = this.target - copter.transform.position;

                    if(offset.magnitude < 10.0f) {
                        SetFlag(Flag.SeekingTargetFlythrough, false);
                        cpu.Interrupt("almost_at_target");

                        if(this.currentInstruction.name == "flythrough") {
                            SetFlag(Flag.ExecutingInstruction, false);
                        }
                    }
                }

                if(HasFlag(Flag.HasTargetAltitude) && HasFlag(Flag.SeekingTargetAltitude)) {
                    if(Mathf.Abs(currentAltitude - this.desiredAltitude) < 1 && copter.rigidBody.velocity.magnitude < 1) {
                        SetFlag(Flag.SeekingTargetAltitude, false);
                        cpu.Interrupt("at_altitude");
                    }
                }

                // this checks if the executing instruction has finished
                if(this.currentInstruction != null) {
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
                    }
                }

                if(cpu.HandlePendingInterrupt()) {
                    // will it be a problem restarting the unfinished instruction afterward?
                    // it definitely will affect "sleep"
                    SetFlag(Flag.ExecutingInstruction, false);
                }

                // some instructions finish immediately, so do them all in one FixedUpdate
                // jmp, call, int, and ret aren't handled in the switch, so we can't get stuck in a loop
                bool isFlightInstruction = true;

                string failReason;
                while(isFlightInstruction && !HasFlag(Flag.ExecutingInstruction)) {
                    // these need to stay synced
                    this.target.y = this.desiredAltitude + GetTerrainHeight(this.target);

                    if(!cpu.Cycle(out this.currentInstruction, out failReason)) {
                        Reset();
                        return;
                    }

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
                            if(TryMapGridToPosition(mapCol, mapRow, out pos1)) {
                                this.target = pos1;
                                this.desiredAltitude = desiredAltitude;
                                SetFlag(Flag.HasTarget, true);
                                SetFlag(Flag.HasTargetAltitude, true);
                                ResetControl();
                            }

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
                        default:
                            isFlightInstruction = false;
                            break;
                    }
                }
            }
        }

        static bool TryMapGridToPosition(string coord, out Vector3 result, bool useCorrection = true) {
             const float gridSize = 146.33f;
             float mapSize = ConVar.Server.worldsize;
            
            if(coord.Length <= 11) {
                var match = Regex.Match(coord, @"^([A-Za-z]{1,2})((?:\.)??|(?:\.[0-9]{1,2})?),([0-9]{1,2})((?:\.)??|(?:\.[0-9]{1,2})?)$");

                if(match.Success && match.Groups.Count == 5) {
                    string letter = match.Groups[1].ToString().ToUpper();
                    string letterFractionStr = match.Groups[2].ToString();
                    string numberWholeStr = match.Groups[3].ToString();
                    string numberFractionStr = match.Groups[4].ToString();

                    float letterFraction = 0;
                    float numberFraction = 0;

                    if(letterFractionStr != "" && letterFractionStr != ".") {
                        letterFraction = float.Parse(letterFractionStr);
                    }

                    if(numberFractionStr != "" && numberFractionStr != ".") {
                        letterFraction = float.Parse(numberFractionStr);
                    }

                    int letterNumber = (letter.Length == 1) ? letter[0] - 'A' : letter[1] + 26 - 'A';
                    float col = letterNumber + letterFraction;
                    float row = float.Parse(numberWholeStr) + numberFraction;

                    float x = col * gridSize - (mapSize / 2);
                    float y = -row * gridSize + (mapSize / 2);

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

        static bool TryMapGridToPosition(float col, float row, out Vector3 result, bool useCorrection = true) {
             const float gridSize = 146.33f;
             float mapSize = ConVar.Server.worldsize;

            float x = col * gridSize - (mapSize / 2);
            float y = -row * gridSize + (mapSize / 2);

            result = new Vector3(x, 0, y);

            if(useCorrection) {
                result += config.gridPositionCorrection;
            }

            return true;
        }

        void OnEntitySpawned(BaseNetworkable entity)
        {
            if(entity is MiniCopter) {
                var miniCopter = entity as MiniCopter;
                ProcessMiniCopter(miniCopter, null);
            }
        }

        private void Cleanup() {
            float time = Time.realtimeSinceStartup;
            
            foreach(var gameObject in GameObject.FindObjectsOfType<MonoBehaviour>()) {
                if(gameObject.name == DroneManager.Guid) {
                    GameObject.Destroy(gameObject);
                    break;
                }
            }

            float timeAfter = Time.realtimeSinceStartup;
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
                    public int addressValue;
                    public int intValue;
                    public float floatValue;
                    public string stringValue;
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

            class Variable {
                public string name;
                public ParamType type;
            }

            Dictionary<string, Param[]> instructionDefs = new Dictionary<string, Param[]> {
                {"label", new Param[] { new Param("name", ParamType.Identifier) }},
                {"isr", new Param[] { new Param("name", ParamType.Identifier) }},
                {"jmp", new Param[] { new Param("label_name", ParamType.Address) }},
                {"call", new Param[] { new Param("label_name", ParamType.Address) }},
                {"int", new Param[] { new Param("isr_name", ParamType.Identifier) }},
                {"ret", new Param[] {  }},
                {"nop", new Param[] {  }},

                {"num", new Param[] { new Param("name", ParamType.Identifier) }},

                {"mov", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"add", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"sub", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"mul", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"div", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"abs", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"sqrt", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},
                {"pow", new Param[] { new Param("dest", ParamType.NumVariable), new Param("rhs", ParamType.Num) }},

                {"je", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
                {"jne", new Param[] { new Param("lhs", ParamType.Num), new Param("rhs", ParamType.Num), new Param("label_name", ParamType.Address) }},
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

                if(!Parse()) {
                    return false;
                }

                var labelAddresses = new Dictionary<string, int>();
                var variables = new Dictionary<string, Variable>();

                // registers r0 - r7
                for(int i = 0; i < 8; i++) {
                    variables.Add("r" + i, new Variable { name = "r" + i, type = ParamType.NumVariable});
                }

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "label") {
                        labelAddresses.Add(instr.args[0].stringValue, i);
                    }

                    if(instr.name == "num") {
                        var arg = instr.args[0];

                        if(variables.ContainsKey(arg.stringValue)) {
                            errors.Add($"line {i}: variable already declared ({arg.stringValue})");
                            return false;
                        }

                        variables.Add(arg.stringValue, new Variable {name = arg.stringValue, type = ParamType.NumVariable});
                    }
                }

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    for(int j = 0; j < instr.args.Count; j++) {
                        var arg = instr.args[j];

                        if(arg.paramType == ParamType.Address) {
                            if(!labelAddresses.TryGetValue(arg.stringValue, out arg.addressValue)) {
                                errors.Add($"line {i}: invalid label ({arg.stringValue})");
                                return false;
                            }
                        }

                        if(arg.argType == ParamType.NumVariable) {
                            if(!variables.ContainsKey(arg.stringValue)) {
                                errors.Add($"line {i}: variable has not been declared ({arg.stringValue})");
                                return false;
                            }
                        }

                        if(arg.paramType == ParamType.Num) {
                            if(arg.argType != ParamType.Num && arg.argType != ParamType.NumVariable) {
                                errors.Add($"line {i}: incompatible argument type ({arg.stringValue}:{arg.argType}) for instruction ({instr.ToString()})");
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
                    errors.Add($"line {line}: invalid argument ({arg}) for ({instr}) spec: {instr} {String.Join(" ", parameters.Select(x => x.name + ':' + x.type))}");
                };

                Action<int, float, int, int, string, ParamType> addArgument = (int index, float floatValue, int intValue, int addressValue, string stringValue, Compiler.ParamType argType) => {
                    compiledInstruction.args.Add(new Instruction.Argument {
                        name = parameters[index].name,
                        paramType = parameters[index].type,
                        argType = argType,
                        rawValue = args[index],
                        addressValue = addressValue,
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

                        if(matchNumVar.Success) {
                            addArgument(i, 0, 0, 0, arg, ParamType.NumVariable);
                        } else if (match.Success) {
                            if(match.Groups[0].ToString() == ".") {
                                fail(arg);
                                return false;
                            }

                            float value;
                            if(!float.TryParse(arg, out value)) {
                                fail(arg);
                                return false;
                            }

                            addArgument(i, value, (int)value, 0, arg, param.type);
                        } else if(matchMapGridCol.Success) {
                            var lettersStr = matchMapGridCol.Groups[1].ToString().ToUpper();
                            var lettersFractionStr = matchMapGridCol.Groups[2].ToString();

                            int lettersWhole = (lettersStr.Length == 1) ? lettersStr[0] - 'A' : lettersStr[1] + 26 - 'A';
                            float lettersFraction = 0.0f;

                            if(lettersFractionStr != "." && lettersFractionStr != "") {
                                if(!float.TryParse(lettersFractionStr, out lettersFraction)) {
                                    fail(arg);
                                    return false;
                                }
                            }

                            var value = lettersWhole + lettersFraction;
                            
                            addArgument(i, value, (int)value, 0, arg, param.type);
                        }
                    }

                    if(param.type == ParamType.Identifier || param.type == ParamType.Address || param.type == ParamType.NumVariable) {
                        var match = Regex.Match(arg, @"^[a-zA-Z_][a-zA-Z0-9_]*$");

                        if(!match.Success) {
                            fail(arg);
                            return false;
                        }
                        
                        addArgument(i, 0, 0, 0, arg, param.type);
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
                        errors.Add($"line {i}: invalid instruction ({instr})");
                        return false;
                    }

                    if(parameters.Length != line.Length - 1) {
                        errors.Add($"line {i}: wrong number of arguments for ({instr}) spec: {instr} {String.Join(" ", parameters.Select(x => x.name + ':' + x.type))}");
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
                var lines = code.Split(new[] {"\r\n", "\r", "\n"}, StringSplitOptions.RemoveEmptyEntries);

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

        void ProcessMiniCopter(MiniCopter miniCopter, StorageContainer existingStorage) {
            StorageContainer storage;

            if(existingStorage == null) {
                var ent = GameManager.server.CreateEntity("assets/prefabs/deployable/woodenbox/woodbox_deployed.prefab");
            
                ent.name = "minicopterdrone.storage";
                ent.Spawn();
                ent.transform.position = miniCopter.transform.position;
                ent.transform.rotation = Quaternion.identity;
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
        }

        void Loaded()
        {
            plugin = this;
            Cleanup();
            var go = new GameObject(DroneManager.Guid);
            droneManager = go.AddComponent<DroneManager>();
            droneManager.plugin = this;
            debugPlayer = BasePlayer.FindByID(config.debugPlayerId);

            foreach(var miniCopter in GameObject.FindObjectsOfType<MiniCopter>()) {
                StorageContainer storage = null;

                // scraptransportheli dervies from MiniCopter
                if(miniCopter.ShortPrefabName != "minicopter.entity") {
                    continue;
                }

                foreach(var ent in miniCopter.children.ToList()) {
                    if(ent is StorageContainer) {
                        if(ent.name == "minicopterdrone.storage") {
                            storage = ent as StorageContainer;
                        }
                    }
                }

                ProcessMiniCopter(miniCopter, storage);
            }
        }

        void Unload()
        {
            Cleanup();
        }

        class PIDController {
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