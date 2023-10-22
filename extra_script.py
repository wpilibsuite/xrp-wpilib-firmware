import os
import re
import shutil
import rjsmin

Import("env")

resSrcDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "src")
resBuildDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "build")
resDir = os.path.join(env.subst("$PROJECT_DIR"), "resources")
minifiedJsDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "minifiedjs")
versionReplaceDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "versionReplace")

# open the VERSION file
versionFile = os.path.join(env.subst("$PROJECT_DIR"), "VERSION")
version = "UNK"
with open(versionFile, "r") as f:
    version = f.read().strip()
print("Version Info: ", version)

print("Resource Dir: ", resDir)

if os.path.exists(resBuildDir):
    shutil.rmtree(resBuildDir)
os.makedirs(resBuildDir)

if os.path.exists(resSrcDir):
    shutil.rmtree(resSrcDir)
os.makedirs(resSrcDir)

if os.path.exists(minifiedJsDir):
    shutil.rmtree(minifiedJsDir)
os.makedirs(minifiedJsDir)

if os.path.exists(versionReplaceDir):
    shutil.rmtree(versionReplaceDir)
os.makedirs(versionReplaceDir)

def genResource(inputFile):
    with open(inputFile, "rb") as f:
        data = f.read()
    fileSize = len(data)

    inputBase = os.path.basename(inputFile)
    funcName = "GetResource_" + re.sub(r"[^a-zA-Z0-9]", "_", inputBase)

    outputFile = os.path.join(resSrcDir, inputBase) + ".cpp"

    with open(outputFile, "wt") as f:
        print("#include <stddef.h>\n#include <string_view>\nextern \"C\" {\nstatic const unsigned char contents[] = { ", file=f, end='')
        print(", ".join("0x%02x" % x for x in data), file=f, end='')
        print(" };", file=f)
        print("const unsigned char* {}{}(size_t* len) {{\n  *len = {};\n  return contents;\n}}\n}}".format("", funcName, fileSize), file=f)

        print("std::string_view {}() {{\n  return std::string_view(reinterpret_cast<const char*>(contents), {});\n}}".format(funcName, fileSize), file=f)

def minifyJs(inputFile):
    with open(inputFile, "r") as f:
        data = f.read()

    inputBase = os.path.basename(inputFile)
    outputFile = os.path.join(minifiedJsDir, inputBase)

    with open(outputFile, "w") as f:
        print(rjsmin.jsmin_for_posers(data), file=f)

    return outputFile

def versionReplace(inputFile):
    with open(inputFile, "r") as f:
        data = f.read()

    inputBase = os.path.basename(inputFile)
    outputFile = os.path.join(versionReplaceDir, inputBase)

    # find replacement
    data = data.replace("%%%VERSION_REPLACE%%%", version)

    with open(outputFile, "w") as f:
        print(data, file=f)

    return outputFile


# generate the version resource string
versionFile = os.path.join(env.subst("$PROJECT_DIR"), "VERSION")
genResource(versionFile)

# Loop through everything in the resources folder
# For each item, generate the resource cpp file
if os.path.exists(resDir):
    for filename in os.listdir(resDir):
        f = os.path.join(resDir, filename)
        if os.path.isfile(f):
            fileExt = os.path.splitext(filename)
            # minify JS files
            if len(fileExt) > 1 and (fileExt[-1]).lower() == ".js":
                genResource(minifyJs(f))
            elif filename == "index.html":
                genResource(versionReplace(f))
            else:
                genResource(f)

# ensure that we add the built stuff to the build path
env.BuildSources(
    os.path.join("$BUILD_DIR", "resources", "build"),
    os.path.join("$BUILD_DIR", "resources", "src")
)
