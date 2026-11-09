import { createFileRoute } from "@tanstack/react-router";

// Pretend car camera for the published preview: /camera/<id> -> /mock/<id>.jpg still frame.
// (In `vite dev` the car-dashboard middleware answers this path first.)
export const Route = createFileRoute("/camera/$id")({
  server: {
    handlers: {
      GET: ({ params, request }) => {
        if (!/^[A-Za-z0-9_-]+$/.test(params.id)) return new Response("Not found", { status: 404 });
        return Response.redirect(new URL(`/mock/${params.id}.jpg`, request.url).toString(), 302);
      },
    },
  },
});
