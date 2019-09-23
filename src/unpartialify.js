let fs = require('fs')

let sourceFile = process.argv[2]
let destFile = process.argv[3]

let sourceCode = fs.readFileSync(sourceFile, 'utf8')
let lines = sourceCode.split(/\n|\r\n/)

let classes = new Map()
let currentClass = null;

for(let i = 0; i < lines.length; i++) {
    let line = lines[i]
    let thing = line.trim().split(/\s+/)

    if(thing[0] == '//unpartialify:begin') {
        currentClass = thing[1]
        classes.set(currentClass, [])
        continue
    }

    if(currentClass != null) {
        if(thing[0] == '//unpartialify:end') {
            currentClass = null
            continue
        }

        classes.get(currentClass).push(line)
    }
}

let destCode = fs.readFileSync(destFile, 'utf8')
lines = destCode.split(/\n|\r\n/)
currentClass = null
let output = []

for(let i = 0; i < lines.length; i++) {
    let line = lines[i]
    let thing = line.trim().split(/\s+/)

    if(thing[0] == '//unpartialify:paste') {
        currentClass = thing[1]
        
        if(!classes.has(currentClass)) {
            console.log(`class not found "${currentClass}, skipping"`)
            output.push(line)
            continue
        }

        let classLines = classes.get(currentClass)

        for(let i = 0; i < classLines.length; i++) {
            output.push(classLines[i])
        }

        continue
    }

    output.push(line)
}

let outputStr = ''

for(let i = 0; i < output.length; i++) {
    outputStr += output[i] + '\n'
}

fs.writeFileSync(destFile, outputStr, 'utf8')