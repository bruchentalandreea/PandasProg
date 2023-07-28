import pandas as pd
import numpy as np

exam_data ={'name': ['Anastasia', 'Dima', 'Katherine', 'James', 'Emily', 'Michael', 'Matthew', 'Laura', 'Kevin', 'Jonas'],
            'score': [12.5, 9, 16.5, np.nan, 9, 20, 14.5, np.nan, 8, 19],
            'attemps': [1, 3, 2, 3, 2, 3, 1, 1, 2, 1],
            'qualify': ['yes', 'no', 'yes', 'no', 'no', 'yes', 'yes', 'no', 'no', 'yes']}

labels = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']

data_frame = pd.DataFrame(exam_data, index=labels)

print(data_frame)

print("\n Transpose dataframe")
print(data_frame.T)

print("\n Types")
print(data_frame.dtypes)

print(" \n Cast attemps to float using dict")
print(data_frame.astype({'attemps':'float'}).dtypes)

print("\n First 5 elements")
print(data_frame.head(5))

print("\n Single elem")
print(data_frame.loc['b', 'name'])

print("\n Insert col")
data_frame.insert(1,"age",[12, 3, 44, 21, 64, 31, 9, 87, 34, 65])
print(data_frame)

print("\n Iterate over dataframe")
for label, content in data_frame.items():
    print(f'label:{label}')
    print(f'content: {content}', sep='\n')

print("\n Iterate over dataframe as namedtuples")
for row in data_frame.itertuples():
    print(row)

#print("\n Return item and drop")
#data_frame.pop('age')
#print(data_frame)

print("\n Return item for given key")
print(data_frame.get('attemps'))

print("\n Query")
print(data_frame.query('score < age'))

