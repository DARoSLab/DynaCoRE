FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.c)

add_library (CDDSrc SHARED ${header} ${source})
