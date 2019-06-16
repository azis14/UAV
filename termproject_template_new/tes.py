x = [32, 38, 20, 2, 9, 25, 34, 21, 27, 4, 5, 7, 6, 19, 22, 11, 30, 39, 16, 23, 36, 1, 15, 28, 8, 29, 0, 14, 13, 12, 31, 10, 17]
y = []


for i in range(len(x)):
	for j in range(i):
		if x[i] < x[j]:
			temp = x[i]
			x[i] = x[j]
			x[j] = temp

for i in x:
	if i not in y:
		y.append(i)



print(x)
print(y)
print(len(y))
