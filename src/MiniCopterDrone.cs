using System;
using Oxide.Core;
using Oxide.Core.Configuration;
using Oxide.Core.Plugins;
using System.Linq;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Text.RegularExpressions;
using UnityEngine;

namespace Oxide.Plugins
{
    [Info("Minicopter Drone", "Andrew", "1.0.0")]
    public class MiniCopterDrone : RustPlugin {
        static MiniCopterDrone plugin = null;
        DroneManager droneManager = null;
        static ConfigData config;
        static Compiler compiler = new Compiler();
        const string calibratePerm = "minicopterdrone.calibrate.allowed";
        static Benchmarker benchmarker = new Benchmarker();

        class Benchmarker {
            class Benchmark {
                public string name;
                public float best;
                public float average;
                public float worst;
                public int samples;
            }

            Dictionary<string, Benchmark> benchmarks = new Dictionary<string, Benchmark>();

            public void Update(string name, float time) {
                Benchmark benchmark;
                if(!benchmarks.TryGetValue(name, out benchmark)) {
                    benchmarks.Add(name, new Benchmark {
                        name = name,
                        best = 0,
                        average = 0,
                        worst = 0,
                        samples = 1
                    });
                } else {
                    benchmark.samples++;
                    benchmark.best = Mathf.Min(benchmark.best, time);
                    benchmark.worst = (benchmark.samples < 10) ? 0 : Mathf.Max(benchmark.worst, time);
                    benchmark.average = (benchmark.samples < 10) ? 0 : benchmark.average - (benchmark.average / benchmark.samples) + (time / benchmark.samples);
                }
            }

            public void Report() {
                foreach(var pair in benchmarks) {
                    var benchmark = pair.Value;
                    Print($"benchmark: {benchmark.name} avg: {benchmark.average}, best: {benchmark.best}, worst: {benchmark.worst}");
                }
            }
        }

        class ConfigData {
            [JsonProperty(PropertyName = "maxProgramInstructions")]
            public int maxProgramInstructions;
            [JsonProperty(PropertyName = "gridPositionCorrection")]
            public Vector3 gridPositionCorrection;
            [JsonProperty(PropertyName = "maxInstructionsPerFixedUpdate")]
            public int maxInstructionsPerFixedUpdate;
            [JsonProperty(PropertyName = "gridSize")]
            public float gridSize;
        }

        protected override void LoadDefaultMessages() {
            lang.RegisterMessages(new Dictionary<string, string> {
                ["must_run_as_player"] = "this command must be run as a player",
                ["need_calibrate_perm"] = "error: you need the {0} permission to use this command",
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
                maxProgramInstructions = 128,
                gridPositionCorrection = new Vector3(1, 0, 75),
                maxInstructionsPerFixedUpdate = 32,
                gridSize = 146.33f
            };

            Config.WriteObject(config, true);
        }

        void Init() {
            config = Config.ReadObject<ConfigData>();
            plugin = this;
        }

        [ConsoleCommand("minicopterdrone.calibrate")]
        void Calibrate(ConsoleSystem.Arg argument) {
            if(!argument.Player()) {
                argument.ReplyWith(lang.GetMessage("must_run_as_player", this));
                return;
            }

            if(!permission.UserHasPermission(argument.Player().UserIDString, calibratePerm)) {
                argument.ReplyWith(lang.GetMessage("need_calibrate_perm", this, argument.Player().UserIDString));
                return;
            }

            Vector3 pos;
            if(TryMapGridToPosition(argument.Args[0], out pos, false)) {
                var actualPos = argument.Player().transform.position;
                config.gridPositionCorrection = new Vector3(actualPos.x, 0, actualPos.z) - pos;
                Config.WriteObject(config, true);
                argument.ReplyWith(lang.GetMessage("grid_pos_corr_saved", this, argument.Player().UserIDString));
            } else {
                argument.ReplyWith(lang.GetMessage("calibrate_invalid_arg", this, argument.Player().UserIDString));
            }
        }

        static void Print(string str) {
            Interface.Oxide.LogInfo(str, new object[]{});
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

        StorageContainer ProcessMiniCopter(MiniCopter miniCopter, StorageContainer existingStorage) {
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

            return storage;
        }

        void Loaded() {
            permission.RegisterPermission(calibratePerm, this);
            var go = new GameObject(DroneManager.Guid);
            droneManager = go.AddComponent<DroneManager>();

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

            var compiler = new Compiler();
            var cpu = new DroneCPU();

            bool success = compiler.Compile(@"
            label top
            num reason

            num x; num y; num z
            num a; num b; num c

            mov reason 42
            mov x 42
            jne x 42 failed
            #print x x

            mov reason 0
            mov x 1
            mov y 2
            mov z 3

            jne x 1 failed
            jne y 2 failed
            jne z 3 failed
            jna x 1 failed
            jna y 2 failed
            jna z 3 failed

            mov reason 1
            push x
            push y
            push z

            pop r0
            pop r1
            pop r2

            jne r0 3 failed
            jne r1 2 failed
            jne r2 1 failed

            mov reason 2
            je 1 1.1 failed
            ja 1 1.1 failed
            jl 1.1 1 failed
            jle 1.1 1 failed
            jg 1 1.1 failed
            jge 1 1.1 failed

            mov reason 3
            #oops

            mov reason 4
            add 1 2
            jne rslt 3 failed
            sub 1 2
            jne rslt -1 failed
            mul 7 7
            jne rslt 49 failed
            div 27 3
            jne rslt 9 failed
            pow 5 2
            jne rslt 25 failed
            min 3 2
            jne 2 rslt failed
            max 3 2
            jne rslt 3 failed

            mov reason 5
            sqrt 49
            jne rslt 7 failed
            abs -1
            jne rslt 1 failed
            floor 1.9999
            jne rslt 1 failed
            ceil 1.0001
            jne rslt 2 failed
            round 1.4999
            jne rslt 1 failed
            sign -3.14
            jne rslt -1 failed

            mov reason 6
            push 1
            push 2
            push 3
            call vec_length
            #print length r0

            #3.74165738677394
            mov reason 7
            jna r0 3.741657 failed

            mov reason 8
            jne r0 3.74165738 failed

            mov reason 9
            lerp 1 2 0.5
            #jne rslt 1.5 failed
            
            jmp end

            label failed
                print failed reason

            label vec_length
                num sum
                mov sum 0
                pop r0
                pop r1
                pop r2
                mul r0 r0
                add sum rslt
                mov sum rslt
                mul r1 r1
                add sum rslt
                mov sum rslt
                mul r2 r2
                add sum rslt
                mov sum rslt
                sqrt sum
                mov r0 rslt
                ret

            label end
            #print success 0
            jmp top
            ");

            if(!success) {
                foreach(var error in compiler.errors) {
                    Print(error);
                }
            } else {
                cpu.LoadInstructions(compiler.instructions);
                var startTime = Time.realtimeSinceStartup;

                int numCycles = 100000000;
                for(int i = 0; i < numCycles; i++) {
                    string reason;
                    Compiler.Instruction instruction;
                    if(cpu.Cycle(out instruction, out reason)) {

                    } else {
                        Print(reason);
                        break;
                    }
                }
                
                var endTime = Time.realtimeSinceStartup;
                Print($"elapsed: {numCycles} in {endTime - startTime}s ({numCycles / (endTime - startTime)} instructions/s)");
                // elapsed: 100000000 in 21.91016s (4564094 instructions/s)
                // elapsed: 100000000 in 14.94202s (6692537 instructions/s)
            }

            // if the plugin crashes or something before Unload is called
            /*var list2 = GameObject.FindObjectsOfType<MonoBehaviour>();
            for(int i = list2.Length - 1; i >= 0; i--) {
                var ent = list2[i];
                if(ent.name == DroneManager.Guid) {
                    GameObject.Destroy(ent);
                }
            }*/

            /*var compiler = new Compiler();
            var success = compiler.Compile(@"
            #droneasm
            startengine
            settarget -1 0 0 0
            setalt -1 50
            setpitch 35
            flythrough

            label loop
                jmp loop
            ");

            if(!success) {
                foreach(var error in compiler.errors) {
                    Print(error);
                }

                return;
            }

            for(int i = 0; i < 50; i++) {
                var position = new Vector3(UnityEngine.Random.Range(-500, 500), 0, UnityEngine.Random.Range(-500, 500));
                position.y = TerrainMeta.HeightMap.GetHeight(position) + 50;
                var miniCopter = GameManager.server.CreateEntity("assets/content/vehicles/minicopter/minicopter.entity.prefab", position) as MiniCopter;
                miniCopter.Spawn();
                var storage = ProcessMiniCopter(miniCopter, null);
                var drone = droneManager.AddDrone(miniCopter, storage);
                drone.cpu.LoadInstructions(compiler.instructions);
                drone.SetFlag(Drone.Flag.EngineOn, true);
                drone.active = true;
            }*/
        }

        void Unload() {
            var list = GameObject.FindObjectsOfType<MiniCopter>();
            for(int i = list.Length - 1; i >= 0; i--) {
                list[i].Kill();
            }

            if(droneManager) {
                GameObject.Destroy(droneManager);
            }
        }

        //unpartialify:paste DroneManager

        //unpartialify:paste Drone

        //unpartialify:paste DroneCPU

        //unpartialify:paste Compiler
        
        //unpartialify:paste PIDController
    }
}