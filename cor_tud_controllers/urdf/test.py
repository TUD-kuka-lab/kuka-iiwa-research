import subprocess

bashCommand = "echo CMAKE_PREFIX_PATH = $CMAKE_PREFIX_PATH"
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
print(output)
