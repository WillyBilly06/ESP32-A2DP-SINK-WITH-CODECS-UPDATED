import os
import shutil
from os import listdir
from glob import glob

class Converter:
    # Provide access to the repository class
    def setRepository(self, repo):
        self.repository = repo
        self.ignoreList = ["/mips/","/arm/","/ppc/","/x86/","/win32/","/fuzzer/","wavreader" ]

    # Ignore Processor specific implementations
    def isFileToIgnore(self, fileName):
        for ignore in self.ignoreList:
            if ignore in fileName:
                return True
        return False

    # Move file to /src directory and remove /include and /src from path
    def targetFileName(self, fileName):
        result = fileName
        result = result.replace("/include","")
        result = result.replace("/src","",1)
        result = result.replace('original/','src/',1)
        return result

    # Includes must not use the leading src/
    def targetIncludeFileName(self, fileName):
        result = self.targetFileName(fileName)
        result = result.replace('src/','',1)
        return result

    # checks if it is an include line
    def isIncludeLine(self, line):
        return line.strip().startswith('#include "')

    # replace the include by adding the subproject prefix
    def replaceInclude(self, line, targetFile ):
        result = line
        pos = targetFile.find("/")
        subproject = targetFile[0:pos]+"/"

        start = line.find("\"")
        end = line.find("\"",start+1)
        if start>0 and end>0:
            includeName = line[start+1:end]
            newIncludeName = self.repository.findIncludeName(includeName, subproject)
            print( "-", includeName, "->", newIncludeName)
            result = "#include \"" + newIncludeName + "\"\n"
        return result

    # convert the file by copying it from the original directory to src and ajusting the includes
    def convert(self, fileSourceName):
        if not self.isFileToIgnore(fileSourceName):
            targetName = self.targetFileName(fileSourceName)
            print("Processing ", fileSourceName, "->", targetName)
            filePath = targetName[0:targetName.rindex("/")]
            os.makedirs(filePath, exist_ok=True)

            file = open(fileSourceName,"rt")
            lines = file.readlines()
            file.close()

            file = open(targetName, "wt")
            for line in lines:
                out = line
                if self.isIncludeLine(line):
                    out = self.replaceInclude(line, targetName)
                file.write(out)
            file.close()

        else:
            print('Ignoring file: ', fileSourceName)


class Repository:
    def __init__(self):
        self.setupWorkingDirectory()
        self.setupGit()
        self.setupDocumentation()
        self.converter = Converter()
        self.converter.setRepository(self)

        self.sourceCppFiles = self.findFiles('original','*.cpp')
        self.sourceHFiles = self.findFiles('original', '*.h') 
        self.setupIncludes()

    # find all files using the matching criterial in the defined subdirectory
    def findFiles(self, path, match):
        return [y for x in os.walk(path) for y in glob(os.path.join(x[0], match))]
    
    # Sets the working directory to the root of the project
    def setupWorkingDirectory(self):
        if os.getcwd().find("/arduino-fdk-aac/")>0:
            pos = os.getcwd().index("/arduino-fdk-aac")
            newPath = os.getcwd()[0:pos+16]
            print("-> new path: ", newPath)
            os.chdir(newPath)

        if not os.getcwd().endswith("/arduino-fdk-aac"):
            os.chdir("arduino-fdk-aac")

        print("Directory: ", os.getcwd())

    # Get the files from Github
    def setupGit(self):
        if os.path.exists("original"):
            cmd = "git -C ./original pull"
        else:
            cmd = "git clone https://github.com/mstorsjo/fdk-aac.git original"

        print(cmd)
        os.system(cmd)

    # Doxygen
    def setupDocumentation(self):
        if not os.path.exists("documentation/html"):
            os.system("doxygen utils/doxyfile")

    # Collect all relevant include paths
    def setupIncludes(self):
        self.headerIncludeNames = []
        for file in self.sourceHFiles:
            newFile = self.converter.targetIncludeFileName(file)
            if not self.converter.isFileToIgnore(newFile):
                self.headerIncludeNames.append(newFile)

    def sourceFiles(self):
        return self.sourceCppFiles

    def headerFiles(self):
        return self.sourceHFiles

    def includeFiles(self):
        return self.headerIncludeNames

    def findIncludeName(self, includeFileNameToFind, preferredDirectory):
        count = 0
        result = includeFileNameToFind
        list = []
        for fileName in self.headerIncludeNames:
            if fileName.endswith(preferredDirectory+includeFileNameToFind):
                return fileName
        for fileName in self.headerIncludeNames:
            if fileName.endswith("/"+includeFileNameToFind):
                count = count + 1
                result = fileName
                list.append(fileName)

        if count!=1:
            print("==> issue: ",includeFileNameToFind, count, list)
        return result


    def applyPatches(self):
        os.system('/usr/bin/patch --verbose src/libAACenc/aacenc.cpp utils/patches/aacenc.patch')
        os.system('/usr/bin/patch --verbose src/libAACdec/usacdec_const.h utils/patches/usacdec_const.patch')

    def processAllFiles(self):
        for fileName in self.headerFiles():
            self.converter.convert(fileName)
        for fileName in self.sourceFiles():
            self.converter.convert(fileName)

        self.applyPatches()


if __name__ == "__main__":
    r = Repository()
    r.processAllFiles()
