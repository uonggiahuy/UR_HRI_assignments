import os,signal,subprocess,sys,time
p=subprocess.Popen(['gdb','-batch','-ex','set pagination off','-ex','run','-ex','thread apply all bt','-ex','quit','--args']+sys.argv[1:])
try:
 time.sleep(15)
 p.send_signal(signal.SIGINT)
 p.wait(timeout=15)
finally:
 if p.poll() is None:p.kill();p.wait()
