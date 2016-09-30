from matplotlib import pyplot as plt
from networkx import Graph, draw_networkx


def repr_action(action, i):
    return "{!r}_{}".format(action, i)


def visualise_plan(plan, filename):
    graph = Graph()

    for i, step in enumerate(plan.plan_steps):
        name = repr_action(step, i)
        graph.add_node(name)

    if len(plan.plan_steps) > 1:
        it = iter(plan.plan_steps)
        next(it)

        i = 0
        for step, next_step in zip(plan.plan_steps, it):
            name = repr_action(step, i)
            next_name = repr_action(next_step, i + 1)
            i += 1
            graph.add_edge(name, next_name)

    plt.axis('off')
    draw_networkx(graph)
    plt.savefig(filename)
    plt.close()
