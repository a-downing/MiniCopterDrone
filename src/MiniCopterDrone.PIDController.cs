namespace Oxide.Plugins
{
    public partial class MiniCopterDrone
    {
        //unpartialify:begin PIDController
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
        //unpartialify:end
    }
}