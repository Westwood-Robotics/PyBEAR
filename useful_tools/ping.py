from pybear import Manager

# bear = Manager.BEAR(port="/dev/UB02B", baudrate=8000000)
bear = Manager.BEAR(port="COM18", baudrate=8000000)
bear_list = []
found = False
for i in range(0, 8):
    m_id = i
    print("Pinging BEAR with ID %d" % m_id)
    data = bear.ping(m_id)
    if data:
        print("Found BEAR with ID %d." % m_id)
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found. And their IDs are:\n" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")
