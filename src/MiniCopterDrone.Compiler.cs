using System;
using System.Linq;
using System.Collections.Generic;

namespace Oxide.Plugins
{
    public partial class MiniCopterDrone
    {
        //unpartialify:begin Compiler
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
                {"print", new Param[] { new Param("string", ParamType.Identifier), new Param("num", ParamType.Num) }},
                
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

                        if(matchNumVar.Success) {
                            addArgument(i, 0, 0, arg, ParamType.NumVariable);
                        } else if(match.Success) {
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
        //unpartialify:end
    }
}