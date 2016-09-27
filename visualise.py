from networkx import Graph, draw_networkx
from matplotlib import pyplot as plt


def visualise_plan(plan, filename):
    graph = Graph()
    for step in plan.plan_steps:
        graph.add_node(step.action)

    if len(plan.plan_steps) > 1:
        it = iter(plan.plan_steps)
        next(it)

        for step, next_step in zip(plan.plan_steps, it):
            graph.add_edge(step.action, next_step.action)

    plt.axis('off')
    draw_networkx(graph)
    plt.savefig(filename)
    plt.close()
