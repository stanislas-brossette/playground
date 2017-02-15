fp = open("entry.txt")
int0 = 0
int1 = 0
list0 = []
list1 = []
for i, line in enumerate(fp):
    if i == 0:
        int0 = int(float(line))
    if i == 1:
        int1 = int(float(line))
    elif i == 2:
        list0 = [int(el) for el in line.split(' ')]
    elif i == 3:
        list1 = [int(el) for el in line.split(' ')]
    elif i>=4:
        break
fp.close()

print("Result:")
s = "Entier 0: " + str(int0)
print(s)
s = "Entier 1: " + str(int1)
print(s)
s = "Liste 0: " + str(list0)
print(s)
s = "Liste 1: " + str(list1)
print(s)
