Import("env")


def generate(source, target, env):
    env.Execute("cp -r .pio/libdeps/release/qp-arduino/libraries/qpn_avr/ lib/")
    env.Execute("qm  sensor/sensor.qm -d -c")
    env.Execute("mkdir -p src")
    env.Execute("rm -rf src/*.ino")
    env.Execute("cp sensor/*.ino src/")


env.AlwaysBuild(env.Alias("generate", None, generate))
