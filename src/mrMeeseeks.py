

















                                                                                                            class MrMeeseeks:

                                                                                                                def __init__(self):
                                                                                                                    self.name = "I'm Mr. Meeseeks!"
                                                                                                                    self.task = "Look at me!"
                                                                                                                    self.taskComplete = "Incomplete"

                                                                                                                def requestTask(self, task):
                                                                                                                    self.task = task
                                                                                                                    possible = self.isTaskPossible(task)

                                                                                                                    if possible == True:
                                                                                                                        self.taskComplete = "Complete"
                                                                                                                    else:
                                                                                                                        self.taskComplete = "Kill Jerry"

                                                                                                                def isTaskPossible(self, task):
                                                                                                                    if task == "Take two strokes off Jerry's golf game":
                                                                                                                        return False
                                                                                                                    else:
                                                                                                                        return True
