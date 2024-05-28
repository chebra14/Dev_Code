import csv

Output_to = 'Results/Output_Test_1.csv'

with open(Output_to, mode='w', newline='') as file:
    writer = csv.writer(file)

def add_row(csv_data):
    with open(Output_to, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_data)