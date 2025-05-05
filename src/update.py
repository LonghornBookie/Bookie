import _sqlite3

# Following LC System: PS3608.O623 I84 2016 for It ends with us / Colleen Hoover
# PS = American Literature
# 3608 - unique id
# .O623 - Cutter Sanborn again
# I84 - Cutter number TITLE, Cutter Sanborn table
# 2016 - Year published

# All 

def update(request: str, writer: str) -> tuple: 
    try:
        # Connect to DB and create cursor
        sqliteConnection = _sqlite3.connect('library.db')
        cursor = sqliteConnection.cursor()

        # Write a query to execute
        query = 'SELECT book_address FROM books WHERE author LIKE "' + writer + '" AND title LIKE "' + request + '";'
        cursor.execute(query)

        found = False

        # Fetch and output result
        result = cursor.fetchall()

        if len(result) > 0:
            found = True

        # Close cursor
        cursor.close()

        return [s[0].strip() for s in result],found

    # Error has occured FATAL
    except _sqlite3.Error as error:
        print('Error occured: ', error)

    # Close connection
    finally:
        if sqliteConnection:
            sqliteConnection.close()
            print('Connection properly closed')

# update()