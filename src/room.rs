use ndarray::prelude::*;
use nannou::prelude::*;
use nannou::image::io::Reader as ImageReader;
fn main() {
    let x: Array2<f64> = Array::zeros((3,3));
    println!("{}", x);
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
    texture: wgpu::Texture,
}

fn model(app: &App) -> Model {
    app.new_window()
        .title("Cram")
        .size(1200+20, 567+20)
        .event(event)
        .view(view)
        .build()
        .unwrap();

    let assets = app.assets_path().unwrap();
    let img_path = assets.join("maps").join("floor.jpg");
    let texture = wgpu::Texture::from_path(app, img_path.clone()).unwrap();
    let img = ImageReader::open(img_path).unwrap().decode().unwrap().to_rgb8();
    // let pixels = img.as_raw().to_vec();
    // let matrix = Array::from_shape_vec((img.height() as usize, img.width() as usize, 3 as usize), pixels).unwrap();
    // println!("{:?}", matrix.shape());

    Model {
        mouse_pos: pt2(0.0, 0.0),
        texture,
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {

}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        }
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    // Prepare to draw.
    let draw = app.draw();

    // Clear the background to purple.
    draw.background().color(WHITE);
    draw.texture(&model.texture);
    // println!("{}", model.mouse_pos);

    draw.to_frame(app, &frame).unwrap();
}