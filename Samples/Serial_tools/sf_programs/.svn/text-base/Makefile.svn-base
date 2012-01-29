all: serial_msg.py 

serial_msg.py: serial_msg.h
	mig python -python-classname=$(@:.py=) $< $(@:.py=) -o $@
