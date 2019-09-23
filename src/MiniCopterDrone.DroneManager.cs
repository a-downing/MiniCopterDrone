using System.Linq;
using System.Collections.Generic;

namespace Oxide.Plugins
{
    public partial class MiniCopterDrone
    {
        //unpartialify:begin DroneManager
        class DroneManager : MonoBehaviour {
            public const string Guid = "918729b6-ca44-46c6-8ad6-1722abff10d4";
            Dictionary<int, Drone> drones = new Dictionary<int, Drone>();

            public static HashSet<int> activeFrequencies = new HashSet<int>();
            public static HashSet<int> risingEdgeFrequencies = new HashSet<int>();
            public static HashSet<int> fallingEdgeFrequencies = new HashSet<int>();
            List<int> removeActiveList = new List<int>();
            List<int> removeDroneList = new List<int>();

            void FixedUpdate() {
                var startTime = Time.realtimeSinceStartup;
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
                        drone.FixedUpdate();
                    }
                }

                foreach(var id in removeDroneList) {
                    drones.Remove(id);
                }
            }

            public Drone AddDrone(MiniCopter miniCopter, StorageContainer storage) {
                var drone = new Drone();
                drone.miniCopterRef.Set(miniCopter);
                drone.instanceId = miniCopter.GetInstanceID();
                drone.storage = storage;
                drone.manager = this;
                drones.Add(miniCopter.GetInstanceID(), drone);
                return drone;
            }

            public bool OnItemAddedOrRemoved(MiniCopter miniCopter, StorageContainer storage, Item item, bool added) {
                Drone drone = null;

                if(!drones.TryGetValue(miniCopter.GetInstanceID(), out drone)){
                    drone = AddDrone(miniCopter, storage);
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
        //unpartialify:end
    }
}