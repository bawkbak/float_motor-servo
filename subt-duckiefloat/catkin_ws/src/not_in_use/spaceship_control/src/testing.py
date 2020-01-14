from dagu_wheels_driver import DaguWheelsDriver

a = DaguWheelsDriver()
a.setThrottle(180)

while(True):
	b = raw_input()
	if b == "a":
		a.setThrottle(0)
	else:
		a.setThrottle(100)
