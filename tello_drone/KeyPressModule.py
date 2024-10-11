import pygame


def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("Control Window")


def getKey(keyName):
    ans = False

    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans


def main():
    if getKey("LEFT"):
        print("Left Key Pressed")
    elif getKey("RIGHT"):
        print("Right Key Pressed")
    if getKey("UP"):
        print("Up Key Pressed")
    if getKey("DOWN"):
        print("Down Key Pressed")
    
    if getKey("p"):
        pygame.display.quit()
        pygame.quit()
    


if __name__ == '__main__':
    init()
    while True:
        main()
