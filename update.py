import _sqlite3

def update(request, writer): 
    try:
        # Connect to DB and create cursor
        sqliteConnection = _sqlite3.connect('library.db')
        cursor = sqliteConnection.cursor()

        # Write a query to execute
        query = 'SELECT location FROM books WHERE author=' + writer + ' AND title=' + request + ';'
        cursor.execute(query)

        # Following LC System: PS3608.O623 I84 2016 for It ends with us / Colleen Hoover
        # PS = American Literature
        # 3608 - unique id
        # .O623 - Cutter Sanborn again
        # I84 - Cutter number TITLE, Cutter Sanborn table
        # 2016 - Year published

        # Fetch and output result
        result = cursor.fetchall()
        print('Table: {}'.format(result))

        # Close cursor
        cursor.close()

        return # Marked location, send to ROS2 driver

    # Error has occured FATAL
    except _sqlite3.Error as error:
        print('Error occured: ', error)

    # Close connection
    finally:
        if sqliteConnection:
            sqliteConnection.close()
            print('Connection properly closed')

update()
