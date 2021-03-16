from pybear import Manager

# bear = Manager.BEAR(port="/dev/UB02B", baudrate=8000000)
bear = Manager.BEAR(port="COM7", baudrate=8000000)
bear_list = []
found = False
start_ID = 0  # The ID to start the search with.
end_ID = 8  # The ID to end the search with
for i in range(start_ID, end_ID+1):
    m_id = i
    print("Pinging BEAR with ID %d" % m_id)
    data = bear.ping(m_id)[0]
    if data:
        print("Found BEAR with ID %d." % m_id)
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found. And their IDs are:\n" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")
