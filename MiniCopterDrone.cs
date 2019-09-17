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
    [Info("MiniCopterDrone", "Andrew", "1.0")]
    public class MiniCopterDrone : RustPlugin
    {
        static MiniCopterDrone plugin;
        static ulong andrewId = 76561198872570126;
        static public BasePlayer andrew = null;
        DroneManager droneManager = null;
        static ConfigData config;

        class ConfigData {
            [JsonProperty(PropertyName = "maxProgramLength")]
            public int maxProgramLength;
            [JsonProperty(PropertyName = "gridPositionCorrection")]
            public Vector3 gridPositionCorrection;
        }

        protected override void LoadDefaultConfig() {
            var config = new ConfigData {
                maxProgramLength = 1024,
                gridPositionCorrection = new Vector3(0.44f, 0, -74.47f)
            };

            Config.WriteObject(config, true);
        }

        void Init() {
            config = Config.ReadObject<ConfigData>();
        }

        /*[ConsoleCommand("minicopterdrone.gridtopos")]
        void GridToPos(ConsoleSystem.Arg argument) {
            Vector3 pos;
            bool success = TryMapGridToPosition(argument.Args[0], out pos);

            if(success) {
                argument.ReplyWith($"{pos.x.ToString("0.000")}, {pos.z.ToString("0.000")}");
            } else {
                argument.ReplyWith("failed");
            }
        }*/

        [ConsoleCommand("minicopterdrone.calibrate")]
        void Calibrate(ConsoleSystem.Arg argument) {
            Vector3 pos;
            bool success = TryMapGridToPosition(argument.Args[0], out pos, false);

            var actualPos = argument.Player().transform.position;
            var error = new Vector3(actualPos.x, 0, actualPos.z) - pos;

            if(success) {
                argument.ReplyWith($"set gridPositionCorrection to this in the config file x: {error.x}, y: {error.y}, z: {error.z}");
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

            Dictionary<Action, float> delayedActions = new Dictionary<Action, float>();

            public void DoAction(Action action, float fixedTime) {
                delayedActions.Add(action, fixedTime);
            }

            void FixedUpdate() {
                float startTime = Time.realtimeSinceStartup;

                foreach(var action in delayedActions.ToArray()) {
                    if(Time.fixedTime > action.Value) {
                        action.Key();
                        delayedActions.Remove(action.Key);
                    }
                }

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

                float endTime = Time.realtimeSinceStartup;
                float elapsedTime = endTime - startTime;
                //plugin.SendReply(andrew, $"DroneManager.FixedUpdate time: {elapsedTime.ToString("0.000000")}s");
            }

            public bool OnItemAddedOrRemoved(MiniCopter miniCopter, StorageContainer storage, Item item, bool added) {
                Print($"added: {added} {item.info.shortname}");

                Drone drone = null;

                if(!drones.TryGetValue(miniCopter.GetInstanceID(), out drone)){
                    drone = new Drone();

                    drone.miniCopterRef.Set(miniCopter);
                    drone.instanceId = miniCopter.GetInstanceID();
                    drone.storage = storage;
                    drone.manager = this;

                    drones.Add(miniCopter.GetInstanceID(), drone);
                    Print($"Added drone #{drone.instanceId}");
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
            List<Compiler.Instruction> instructions = null;
            public Dictionary<string, int> isrs = null;
            public Queue<string> interrupts;
            public List<int> picStack;
            int pic = 0;

            public void Reset() {
                pic = 0;
                interrupts = new Queue<string>();
                picStack = new List<int>();
            }

            public void LoadInstructions(List<Compiler.Instruction> instructions) {
                this.instructions = instructions;
                isrs = new Dictionary<string, int>();
                interrupts = new Queue<string>();
                picStack = new List<int>();
                pic = 0;

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "isr") {
                        isrs.Add(instr.args[0].stringValue, i);
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

            public bool Cycle(out Compiler.Instruction instr) {
                if(instructions == null || pic < 0 || pic >= instructions.Count) {
                    instr = null;
                    return false;
                }

                instr = instructions[pic++];

                if(instr.name == "label" || instr.name == "isr") {
                    return Cycle(out instr);
                }

                if(instr.name == "jmp") {
                    Jump(instr.args[0].addressValue);
                }

                if(instr.name == "call") {
                    Call(instr.args[0].addressValue);
                }

                if(instr.name == "int") {
                    Interrupt(instr.args[0].stringValue);
                    HandlePendingInterrupt();
                }

                if(instr.name == "ret") {
                    if(!Ret()) {
                        return false;
                    }
                }

                return true;
            }
        }

        class Drone {
            public DroneManager manager;
            public EntityRef miniCopterRef;
            public int instanceId;
            Compiler compiler = new Compiler();
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
            }

            public Flag flags;

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
                    var lines = item.text.ToLower().Split(new[] {"\r\n", "\r", "\n"}, StringSplitOptions.None);
                    const int maxProgramLength = 1024;

                    if(lines.Length >= 1) {
                        if(lines[0] == "#droneasm") {
                            if(added) {
                                if(lines.Length < maxProgramLength) {
                                    Reset();
                                    bool success = compiler.Compile(item.text);

                                    if(!success) { 
                                        item.text += "\n" + String.Join("\n", compiler.errors.Select(x => "#[error] " + x));
                                    } else {
                                        cpu.LoadInstructions(compiler.instructions);
                                        active = true;
                                    }
                                } else {
                                    item.text += "\n" + $"#[error] max program length is {maxProgramLength} bytes";
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

                //andrew.SendConsoleCommand("ddraw.arrow", Time.fixedDeltaTime, Color.red, copter.transform.position, copter.transform.position + (thrust / copter.engineThrustMax) * 5, 0.25);

                copter.rigidBody.AddForce(thrust, ForceMode.Force);
            }

            void CyclicControl(Vector3 desired) {
                var copter = miniCopterRef.Get(true) as MiniCopter;
                var pv = copter.transform.rotation * Vector3.up;
                var control = cyclicController.Update(desired, pv, Time.fixedDeltaTime);

                //andrew.SendConsoleCommand("ddraw.arrow", Time.fixedDeltaTime, Color.green, copter.transform.position, copter.transform.position + control * 4, 0.25);

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
                    this.desiredAltitude -= this.landingSpeed * Time.fixedDeltaTime;
                }

                if(HasFlag(Flag.EngineOn)) {
                    //copter.ApplyWheelForce(copter.frontWheel, 1, 0, 1);
                    //copter.ApplyWheelForce(copter.leftWheel, 1, 0, 0);
                    //copter.ApplyWheelForce(copter.rightWheel, 1, 0, 0);

                    if(HasFlag(Flag.Flying)) {
                        if(HasFlag(Flag.HasTargetAltitude)) {
                            ControlAltitude(desiredAltitude, currentAltitude);
                        }

                        Vector3 copterUp = Vector3.up;

                        if(HasFlag(Flag.HasTarget)) {
                            this.target.y = this.desiredAltitude + GetTerrainHeight(this.target);
                            copterUp = HeadingControl(this.target, this.desiredPitch);
                        }

                        //andrew.SendConsoleCommand("ddraw.arrow", Time.fixedDeltaTime, Color.blue, copter.transform.position, copter.transform.position + copterUp * 6, 0.25);
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

                /*var flagValues = Enum.GetValues(typeof(Flag));
                var flagNames = Enum.GetNames(typeof(Flag));
                var setFlags = new List<string>();

                for(int i = 0; i < flagValues.Length; i++) {
                    var val = (Flag)flagValues.GetValue(i);

                    if(HasFlag(val)) {
                        setFlags.Add(flagNames[i]);
                    }
                }*/

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

                if(cpu.HandlePendingInterrupt() && !HasFlag(Flag.ExecutingInstruction)) {
                    // will it be a problem restarting the unfinished instruction afterward?
                    // it definitely will affect "sleep"
                    SetFlag(Flag.ExecutingInstruction, false);
                }

                // some instructions finish immediately, so do them all in one FixedUpdate
                // jmp, call, int, and ret aren't handled in FixedUpdate, so we can't get stuck in a loop
                bool isFlightInstruction = true;

                while(isFlightInstruction && !HasFlag(Flag.ExecutingInstruction)) {
                    if(!cpu.Cycle(out this.currentInstruction)) {
                        Reset();
                        return;
                    }

                    switch(this.currentInstruction.name) {
                        case "sleep":
                            var seconds = currentInstruction.args[0].floatValue;
                            
                            // this is to pick up from where we left off after an interrupt
                            // not sure if this is the ideal behavior
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
                                this.target.y = this.desiredAltitude + GetTerrainHeight(this.target);
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
                                SetFlag(Flag.Landing, true);
                                SetFlag(Flag.ExecutingInstruction, true);
                            }

                            break;
                        case "flyto":
                        case "flythrough":
                            var desiredPitch = currentInstruction.args[0].floatValue;

                            if(HasFlag(Flag.HasTarget) || HasFlag(Flag.HasTargetAltitude)) {
                                SetFlag(Flag.Flying, true);
                                SetFlag(Flag.ExecutingInstruction, true);
                                this.desiredPitch = desiredPitch;
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
                        /*case "detonate":
                            Effect.server.Run("assets/prefabs/npc/patrol helicopter/effects/rocket_explosion.prefab", storage.transform.position);
                            Effect.server.Run("assets/bundled/prefabs/fx/explosions/explosion_01.prefab", storage.transform.position);

                            var entities = new List<BaseCombatEntity>();
                            Vis.Entities(storage.transform.position, 5, entities);
                            
                            foreach (var entity in entities) {
                                entity.Hurt(100, Rust.DamageType.Explosion);
                            }

                            break;*/
                        default:
                            isFlightInstruction = false;
                            break;
                    }
                }

                if(!HasFlag(Flag.ExecutingInstruction)) {
                    return;
                }

                // this checks if the executing instruction has finished
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
                    case "land":
                        if(currentAltitude < 2) {
                            SetFlag(Flag.Landing, false);
                            SetFlag(Flag.Flying, false);
                            SetFlag(Flag.ExecutingInstruction, false);
                        }

                        break;
                    case "flyto":
                        if(HasFlag(Flag.HasTarget)) {
                            Vector3 offset = this.target - copter.transform.position;

                            if(offset.magnitude < 1 && copter.rigidBody.velocity.magnitude < 1) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }
                        }

                        if(!HasFlag(Flag.HasTarget)) {
                            if(Mathf.Abs(currentAltitude - this.desiredAltitude) < 1) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }
                        }

                        break;
                    case "flythrough":
                        if(HasFlag(Flag.HasTarget)) {
                            Vector3 offset = this.target - copter.transform.position;

                            if(offset.magnitude < 10) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }
                        }

                        if(!HasFlag(Flag.HasTarget)) {
                            if(Mathf.Abs(currentAltitude - this.desiredAltitude) < 1) {
                                SetFlag(Flag.ExecutingInstruction, false);
                            }
                        }

                        break;
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
                    Puts($"destroying {gameObject.name}");
                    GameObject.Destroy(gameObject);
                    break;
                }
            }

            float timeAfter = Time.realtimeSinceStartup;
            Puts($"Cleanup() elapsed time: {timeAfter - time}");
        }

        class Compiler {
            List<string[]> tokens = new List<string[]>();
            public List<string> errors;
            public List<Instruction> instructions = new List<Instruction>();

            public enum Types {
                Num,
                CharNum,
                Identifier,
                Address
            }

            public class Instruction {
                public class Argument {
                    public string name;
                    public Types type;
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
                    return $"{name} {String.Join(" ", args.Select(x => x.rawValue))}";
                }
            }

            struct Param {
                public Types type;
                public string name;

                public Param(string name, Types type) {
                    this.name = name;
                    this.type = type;
                }
            }

            Dictionary<string, Param[]> instructionDefs = new Dictionary<string, Param[]> {
                {"label", new Param[] { new Param("name", Types.Identifier) }},
                {"isr", new Param[] { new Param("name", Types.Identifier) }},
                {"jmp", new Param[] { new Param("label_name", Types.Address) }},
                {"call", new Param[] { new Param("label_name", Types.Address) }},
                {"int", new Param[] { new Param("isr_name", Types.Identifier) }},
                {"ret", new Param[] {  }},
                {"nop", new Param[] {  }},

                {"startengine", new Param[] {  }},
                {"stopengine", new Param[] {  }},
                {"land", new Param[] { new Param("speed", Types.Num) }},
                {"sleep", new Param[] { new Param("seconds", Types.Num) }},
                {"target", new Param[] { new Param("grid_col", Types.CharNum), new Param("grid_row", Types.Num), new Param("altitude", Types.Num) }},
                {"targetalt", new Param[] { new Param("altitude", Types.Num) }},
                {"targethere", new Param[] {  }},
                {"targetrf", new Param[] { new Param("frequency", Types.Num) }},
                {"flyto", new Param[] { new Param("pitch", Types.Num) }},
                {"flythrough", new Param[] { new Param("pitch", Types.Num) }},
                {"drop", new Param[] { new Param("slot", Types.Num) }},
                {"detonate", new Param[] {  }},
            };

            public bool Compile(string code) {
                errors = new List<string>();
                tokens = new List<string[]>();
                instructions = new List<Instruction>();

                code = Regex.Replace(code, @"#.*$", "", RegexOptions.Multiline);

                Tokenize(code);

                if(!Parse()) {
                    return false;
                }

                var labelAddresses = new Dictionary<string, int>();

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    if(instr.name == "label") {
                        labelAddresses.Add(instr.args[0].stringValue, i);
                    }
                }

                for(int i = 0; i < instructions.Count; i++) {
                    var instr = instructions[i];

                    for(int j = 0; j < instr.args.Count; j++) {
                        var arg = instr.args[j];

                        if(arg.type == Types.Address) {
                            if(!labelAddresses.TryGetValue(arg.stringValue, out arg.addressValue)) {
                                errors.Add($"line {i}: invalid label ({arg.stringValue})");
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

                Action<int, float, int, int, string> addArgument = (int index, float floatValue, int intValue, int addressValue, string stringValue) => {
                    compiledInstruction.args.Add(new Instruction.Argument {
                        name = parameters[index].name,
                        type = parameters[index].type,
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

                    if(param.type == Types.Num) {
                        var match = Regex.Match(arg, @"^[0-9]*[\.]?[0-9]*$");

                        if(!match.Success) {
                            fail(arg);
                            return false;
                        }

                        if(match.Groups[0].ToString() == ".") {
                            fail(arg);
                            return false;
                        }

                        float value;
                        if(!float.TryParse(arg, out value)) {
                            fail(arg);
                            return false;
                        }

                        addArgument(i, value, (int)value, 0, arg);
                    }

                    if(param.type == Types.CharNum) {
                        var match = Regex.Match(arg, @"^([a-zA-Z]{1,2})([\.][0-9]*)?$");

                        if(!match.Success) {
                            fail(arg);
                            return false;
                        }

                        var lettersStr = match.Groups[1].ToString().ToUpper();
                        var lettersFractionStr = match.Groups[2].ToString();

                        int lettersWhole = (lettersStr.Length == 1) ? lettersStr[0] - 'A' : lettersStr[1] + 26 - 'A';
                        float lettersFraction = 0.0f;

                        if(lettersFractionStr != "." && lettersFractionStr != "") {
                            if(!float.TryParse(lettersFractionStr, out lettersFraction)) {
                                fail(arg);
                                return false;
                            }
                        }

                        var value = lettersWhole + lettersFraction;
                        
                        addArgument(i, value, (int)value, 0, arg);
                    }

                    if(param.type == Types.Identifier || param.type == Types.Address) {
                        var match = Regex.Match(arg, @"^[a-zA-Z_][a-zA-Z0-9_]*$");

                        if(!match.Success) {
                            fail(arg);
                            return false;
                        }
                        
                        addArgument(i, 0, 0, 0, arg);
                    }
                }

                instructions.Add(compiledInstruction);
                return true;
            }

            bool Parse() {
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
            Puts("Loaded()");
            plugin = this;

            Cleanup();
            var go = new GameObject(DroneManager.Guid);
            droneManager = go.AddComponent<DroneManager>();
            droneManager.plugin = this;

            andrew = BasePlayer.FindByID(andrewId);
            
            if(andrew) {
                Puts("Found Andrew");
            }

            foreach(var miniCopter in GameObject.FindObjectsOfType<MiniCopter>()) {
                StorageContainer storage = null;

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