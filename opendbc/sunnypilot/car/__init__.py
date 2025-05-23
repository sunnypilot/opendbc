def get_param(params, key):
  for entry in params.entries:
    if entry.key == key:
      return entry.value

  return None
