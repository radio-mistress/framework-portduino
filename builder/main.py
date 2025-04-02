from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
env.Replace(
    AR="ar",
    AS="as",
    CC="gcc",
    CXX="g++",
    OBJCOPY="objcopy",
    RANLIB="ranlib",
    SIZETOOL="size",
)

env.Append(
    CPPDEFINES=["LINUX"],
    CPPPATH=["include"],
    LIBS=["m"],
)