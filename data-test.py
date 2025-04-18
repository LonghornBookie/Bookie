import update as db_connect

result, found = db_connect.update("The Great Gatsby", "F. Scott Fitzgerald")
print(result)