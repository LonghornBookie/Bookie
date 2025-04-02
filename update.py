import _sqlite3

def update(): 
    try:
        # Connect to DB and create cursor
        sqliteConnection = _sqlite3.connect('library.db')
        cursor = sqliteConnection.cursor()

        # Write a query to execute
        query = 'SELECT * FROM books;'
        cursor.execute(query)

        # Fetch and output result
        result = cursor.fetchall()
        print('Table: {}'.format(result))

        # Close cursor
        cursor.close()

    # Error has occured FATAL
    except _sqlite3.Error as error:
        print('Error occured: ', error)

    # Close connection
    finally:
        if sqliteConnection:
            sqliteConnection.close()
            print('Connection properly closed')

update()
