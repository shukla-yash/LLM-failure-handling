'''
Primitive operators defined for rigorous plan definition
'''


class Precondition:
    name: str
    explanation: str 


class Effect:
    name: str
    explanation: str 


class Operator:
    name: str
    explanation: str 
    precondition: Precondition 
    arguments: str | list[str] 
    effect: Effect 


class Plan:
    name: str 
    plan: list[Operator]