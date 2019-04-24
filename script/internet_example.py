class Point():
    def __init__(self, x, y, ):
        self.time = 1
        self.x = x
        self.y = y
        map[self.y][self.x] = self.time
        self.record = [[self.x, self.y]]

    def trace(self):
        map[self.y][self.x] = self.time
        self.record.append([self.x, self.y, ])
    def pathto(self, x, y,):
        while not ((self.x, self.y, ) == (x, y,)):
            self.time += 1

            try:
                dx = int(abs(self.x - x) / (self.x - x))
            except:
                dx = 0
            try:
                dy = int(abs(self.y - y) / (self.y - y))
            except:
                dy = 0
            if not ([self.x - dx, self.y - dy, ] in np.argwhere(map == -1).tolist()
                    or [self.x - dx, self.y - dy, ] in np.argwhere(map == self.time)):
                self.x -= dx
                self.y -= dy
                self.trace()
            elif not ([self.x, self.y - dy, ] in np.argwhere(map == -1).tolist()
                      or [self.x, self.y - dy, ] in np.argwhere(map == self.time)):
                self.y -= dy
                self.trace()
            elif not ([self.x - dx, self.y, ] in np.argwhere(map == -1).tolist()
                      or [self.x - dx, self.y, ] in np.argwhere( map == self.time)):
                self.x -= dx
                self.trace()
            #
            elif not ([self.x + dx, self.y, ] in np.argwhere(map == -1).tolist()
                      or [self.x + dx, self.y, ] in np.argwhere( map == self.time)):
                self.x += dx
                self.trace()
            elif not ([self.x + dx, self.y, ] in np.argwhere(map == -1).tolist()
                      or [self.x + dx, self.y, ] in np.argwhere( map == self.time)):
                self.x += dx
                self.trace()
            elif not ([self.x - dx, self.y + dy, ] in np.argwhere(map == -1).tolist()
                      or [self.x - dx, self.y + dy, ] in np.argwhere( map == self.time)):
                self.x -= dx
                self.y += dy
                self.trace()
            elif not ([self.x + dx, self.y - dy, ] in np.argwhere(map == -1).tolist()
                      or [self.x + dx, self.y - dy, ] in np.argwhere( map == self.time)):
                self.x += dx
                self.y -= dy
                self.trace()
            elif not ([self.x + dx, self.y + dy, ] in np.argwhere(map == -1).tolist()
                      or [self.x + dx, self.y + dy, ] in np.argwhere( map == self.time)):
                self.x += dx
                self.y += dy
                self.trace()

        map[self.y][self.x] = -1
