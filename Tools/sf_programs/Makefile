all: serial_msg.py serial_msg.java

serial_msg.py: serial_msg.h
	mig python -python-classname=$(@:.py=) $< $(@:.py=) -o $@

serial_msg.java: serial_msg.h
	mig java -java-classname=blah.foo.$(@:.java=) $< $(@:.java=) -o $@
