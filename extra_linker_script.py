Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

env.Append(
  LINKFLAGS=[
      "-nostartfiles",
      "-mrelax" 
  ]
)