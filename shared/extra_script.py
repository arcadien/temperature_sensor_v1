Import("env")


def init(source, target, env):
    env.Execute("cp -r .pio/libdeps/release/qp-arduino/libraries/qpn_avr/ lib/")
    env.Execute("qm  sensor/sensor.qm -d -c")
    env.Execute("mkdir -p src")
    env.Execute("rm -rf src/*")
    env.Execute("cp sensor/*.h   src/")
    env.Execute("cp sensor/*.cpp src/")
    env.Execute("cp sensor/*.ino src/")


env.AlwaysBuild(env.Alias("init", None, init))
