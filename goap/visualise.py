from matplotlib import pyplot as plt
from networkx import DiGraph, draw_networkx


def repr_step(step):
    return step.action.__class__.__name__


def look_ahead(iterable):
    sequence = list(iterable)
    shifted_sequence = iter(sequence)
    next(shifted_sequence)
    return zip(sequence, shifted_sequence)


def visualise_plan(plan, filename):
    graph = DiGraph()

    for i, step in enumerate(plan):
        name = repr_step(step)
        graph.add_node(name)

    if len(plan) > 1:
        for i, (step, next_step) in enumerate(look_ahead(plan)):
            name = repr_step(step)
            next_name = repr_step(next_step)
            graph.add_edge(name, next_name)

    plt.axis("off")
    draw_networkx(graph)
    plt.savefig(filename)
    plt.close()
