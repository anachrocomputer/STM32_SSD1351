# ppm2c --- convert an ASCII PPM fle to a C array              2020-10-28
# Copyright (c) 2020 John Honniball. All rights reserved

import sys

def main():
    ppmName = sys.argv[1]
    cArrayName = sys.argv[2]
    imageName = sys.argv[3]

    ppm = open(ppmName, 'r')
    cArray = open(cArrayName, 'w')

    ppmType = ppm.readline()
    ppmComment = ppm.readline()
    ppmDimensions = ppm.readline()
    ppmMaxPixel = ppm.readline()

    cArray.write('const uint16_t %s[64][64] = {\n' % imageName)

    for y in range(64):
        cArray.write('   {')
        
        for x in range(64):
            rStr = ppm.readline()
            gStr = ppm.readline()
            bStr = ppm.readline()
            
            r5 = int(rStr) // 8;
            g6 = int(gStr) // 4;
            b5 = int(bStr) // 8;
            
            rgb565 = (b5 << 11) | (g6 << 5) | r5;
            
            cArray.write("0x%04x" % rgb565)
            
            if x < 63:
                if ((x + 1) % 8) == 0:
                    cArray.write(",\n    ")
                else:
                    cArray.write(", ")
        
        if y < 63:
            cArray.write('},\n') 
        else:
            cArray.write('}\n')
    
    cArray.write('};')
    
    cArray.close()
    ppm.close()
    
    return (0)


if __name__ == '__main__':
    sys.exit(main())

