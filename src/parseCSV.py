import csv

def parseCSV(filename, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%'):
	with open(filename, "r", encoding='utf-8') as csvfile:
		csvFile = csvfile.readlines()
		matrix = []
		maxSize = 0

		for line in csvFile:
			if(line.startswith(commentChar)):

				continue

			row = list(map(float, filter(None, line.rstrip().split(delimiter))))
			matrix.append(row)
			if (len(row) > maxSize):
				maxSize = len(row)

		if(not(returnJagged)):
			for row in matrix:
				row += [fillerValue] * (maxSize - len(row))

		return matrix

def printMatrix(matrix):
	for row in matrix:
		for cell in row:
			print(cell),
		print()

if __name__ == "__main__":
	matrix = parseCSV("data/test.csv")
	printMatrix(matrix)
