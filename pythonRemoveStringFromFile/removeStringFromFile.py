import os

folder = "files/"
folderClean = "cleanFiles/"

for filename in os.listdir(folder):
    infile = folder + filename
    outfile = folderClean + filename

    delete_list = ["Solver error: "]
    fin = open(infile)
    fout = open(outfile, "w+")
    for line in fin:
        for word in delete_list:
            line = line.replace(word, "")
        fout.write(line)
    fin.close()
    fout.close()
