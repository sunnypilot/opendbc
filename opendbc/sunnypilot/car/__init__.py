def get_param(params, key) -> str:
  for param in params:
    if param.key == key:
      return param.value
  return "0"
