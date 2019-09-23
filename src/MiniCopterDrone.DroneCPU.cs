using System;
using System.Linq;
using System.Collections.Generic;

namespace Oxide.Plugins
{
    public partial class MiniCopterDrone
    {
        //unpartialify:begin DroneCPU
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

                if(instr.name == "print") {
                    Print($"cpu print: {instr.args[0].rawValue} {instr.args[1].floatValue}");
                }

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
                        }
                    } catch(ArithmeticException e) {
                        failReason = e.ToString();
                        return false;
                    }
                }

                if(instr.name == "lerp") {
                    float result = Mathf.Lerp(instr.args[0].floatValue, instr.args[1].floatValue, instr.args[2].floatValue);
                    WriteVariable("rslt", result);
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
                    }
                }

                failReason = null;
                return true;
            }
        }
        //unpartialify:end
    }
}