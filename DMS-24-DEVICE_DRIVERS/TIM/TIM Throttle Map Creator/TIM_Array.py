 #
 # TIM_Array.py
 # Created on: Dec 30, 2023
 # Author: Ashton Dudley
 # @brief Tool to convert TIM maps to arrays.
 #


def is_float(element: any) -> bool:
    #If you expect None to be passed:
    if element is None: 
        return False
    try:
        float(element)
        return True
    except ValueError:
        return False

def main():
    exit = False
    
    while not exit:
        print("Enter/Paste your content. Ctrl-D (linux) or Ctrl-Z ( windows ) to save it.")
        contents = []
        while True:
            try:
                line = input()
            except EOFError:
                break
            contents.append(line)
                   
        arrayContents = []
        for i in contents:
            if is_float(i):
                if i.__contains__("."):
                   arrayContents.append(i+"f")
                else:
                   arrayContents.append(i+".0f")
        
        outputArray = "{"
        for i in arrayContents:
            outputArray = outputArray + i + ", "
        outputArray = outputArray[:-1]
        outputArray = outputArray[:-1]
        outputArray = outputArray + "}"
        print(outputArray)

        exit = True
    return

if __name__ == "__main__":
    main()




