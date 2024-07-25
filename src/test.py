def make_str(result_list):
    result_str = ""
    split_char = '//'
    result_str = ""
    try:
        for i in range(len(result_list)):
            result_str = result_str + result_list[i]
            if i < len(result_list)-1:
                result_str += split_char
    except Exception as error:
        print(f"RobotSystem make_str ERROR {error}")
        result_str = "ERROR"
    finally: 
        return result_str 

def main():
    test = ["HOME",'HOME','0','0','0']
    print(make_str(test))

main()