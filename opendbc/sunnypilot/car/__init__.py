def get_param(params: list, key: str) -> str:
  for param in params:
    if param.key == key:
      return param.value
  return "0"
