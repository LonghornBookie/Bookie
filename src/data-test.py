import update as db_connect

result, found = db_connect.update("atomic habits", "james clear")
print(result)