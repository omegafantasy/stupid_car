import json, os, re
from handyllm import OpenAIAPI, PromptConverter
converter = PromptConverter()

def json_check(s: str):
    # greedy search the longest string
    ret = re.search(r'\{.*\}', s, flags=re.DOTALL)
    result = ''
    if ret:
        result = ret[0]
    else:
        print(f"Invalid JSON: {s}")
        # weak fix malformed JSON
        i = s.find('{')
        j = s.find('}')
        if i != -1:
            result =  s[i:] + '\n}'
        elif j != -1:
            result =  '{\n' + s[:j+1]
        elif j == -1:
            result =  '{\n' + s + '\n}'    

    return result

def gen_arg(instruction:str):
    template_path = os.path.join(os.getcwd(), 'gen_arg.txt')
    template = converter.rawfile2chat(template_path)
    new_chat = converter.chat_replace_variables(template, {r'%instruction%': instruction})
    response = OpenAIAPI.chat(
        model='gpt-4',
        messages=new_chat,
        temperature=0.77,
        max_tokens=512,
    ) 
    #arg = json.loads(json_check(response['choices'][0]['message']['content']))
    arg = json_check(response['choices'][0]['message']['content'])
    return arg