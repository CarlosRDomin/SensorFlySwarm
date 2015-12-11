import subprocess as sp
import os


def convertRelPathToRpath(line, dynLibPath, preTabs = ""):
    if line.startswith("lib/"):  # If it starts with "lib/", prepend "@rpath/" to the path
        print preTabs + "Line read:" + line
        oldPath = line.split(" ")[0]
        sp.call(["sudo", "install_name_tool", "-change", oldPath, "@rpath/" + oldPath, dynLibPath])
        print preTabs + "Changed " + oldPath + " to " + "@rpath/" + oldPath


if __name__ == '__main__':
    strToFind = ["CMAKE_INSTALL_PREFIX=", "PYTHON2_PACKAGES_PATH="]
    cvLibPath = ""
    with open("CONFIGURE_preinstall.sh") as f:
        for l in f:
            if l.find(strToFind[0]) >= 0:   # CMAKE_INSTALL_PREFIX indicates the base directory where the dynamic libraries will be installed
                rPath = l.split(strToFind[0])[1].split(" ")[0]  # Remove anything to the left of strToFind, and anything after the first space from there (both inclusive)
            if l.find(strToFind[1]) >= 0:   # PYTHON2_PACKAGES_PATH indicates where cv2.so is located
                cvLibPath = l.split(strToFind[1])[1].split(" ")  # Remove anything to the left of strToFind
                if len(cvLibPath) <= 2:  # There should be at most one space if path is absolute
                    cvLibPath = os.path.expanduser(cvLibPath[0] + "/cv2.so")  # Remove anything after the first space
                else:  # Otherwise, it's a script of the form '$(python -c "from distutils.sysconfig import get_python_lib; aux=get_python_lib(); print(aux)")'
                    aux = {}
                    exec " ".join(cvLibPath[0:-1]).split('"')[1] in aux  # Keep the code inside the quotation marks and execute it
                    cvLibPath = aux["aux"] + "/cv2.so"  # Variable aux should hold the output of the script (site-packages path)

    if cvLibPath == "":
        print "Error: couldn't locate cv2.so!"
    else:
        try:
            sp.call(["sh", "getSudoAccess.sh"])    # Neat script to gain sudo access, asking for passwd if necessary

            print "Analyzing dynamic library " + cvLibPath + "..."
            for l in sp.check_output(["otool", "-L", cvLibPath]).split("\n"):   # For every dynamic library loaded, update the path if necessary
                convertRelPathToRpath(l.lstrip(" \t\r\n"), cvLibPath)

            try:
                sp.call(["sudo", "install_name_tool", "-add_rpath", rPath, cvLibPath])   # Finally, define @rpath as rPath (eg: "/opt/local")
            except Exception: pass  # If same rpath already existed, install_name_tool throws an error, but we can keep going

            print "Finished updating " + cvLibPath + ". Now going to update every library in " + rPath + "/lib/libopencv*..."
            # for f in sp.check_output(["ls", rPath + "/lib/libopencv*"]).split("\n"):    # For every file in @rPath/lib/libopencv*
            for f in os.listdir(rPath + "/lib/"):    # For every file in @rPath/lib/libopencv*
                if f.startswith("libopencv"):
                    libPath = rPath + "/lib/" + f
                    for l in sp.check_output(["otool", "-L", libPath]).split("\n"): # For every dynamic library loaded, update the path if necessary
                        convertRelPathToRpath(l.lstrip(" \t\r\n"), libPath, "\t")
                    try:
                        sp.call(["sudo", "install_name_tool", "-add_rpath", rPath, libPath])    # Finally, define @rpath as rPath (eg: "/opt/local")
                    except Exception: pass  # If same rpath already existed, install_name_tool throws an error, but we can keep going
                    print "-----------------------------------------------------------------------------"
            print "YAAAAY, DONE!!! :)"
        except Exception:
            print "Error"
