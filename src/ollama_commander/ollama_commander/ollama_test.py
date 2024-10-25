import ollama
response = ollama.chat(model='twist', messages=[
  {
    'role': 'user',
    'content': 'go straight',
    'format': 'json'
  },
])
print(response['message']['content'])