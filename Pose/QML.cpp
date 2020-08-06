#include "Qml_integrator.hpp"

 
/*
void GUI::receiveFrame(cv::Mat image)
{
	cv::Mat dp_image;
	dp_image = cv::Scalar(0);

	if (!image.empty())
		cv::cvtColor(image, dp_image, COLOR_BGR2RGB);
	qDebug() << GUI_MSG << "image convert";
	QImage output((const unsigned char*)dp_image.data, dp_image.cols, dp_image.rows, QImage::Format_RGB888);
	ui.Camera->setPixmap(QPixmap::fromImage(output));
	ui.Camera->update();

}

void GUI::receiveFrame2(cv::Mat image)
{
	cv::Mat dp_image;
	dp_image = cv::Scalar(0);

	if (!realsense->Color_image.empty())
		cv::cvtColor(realsense->Color_image, dp_image, COLOR_BGR2RGB);

	QImage output((const unsigned char*)dp_image.data, dp_image.cols, dp_image.rows, QImage::Format_RGB888);
	ui.Camera_2->setPixmap(QPixmap::fromImage(output));
	ui.Camera_2->update();
}


void GUI::on_Camera_Switch_clicked(bool toggled)
{
	if (toggled == true)
	{

	}
	else if (toggled == false)
	{
		ui.Face_Switch->setChecked(toggled);
		ui.Pose_Switch->setChecked(toggled);
		image->terminate();
	}
}

void GUI::on_Face_Switch_clicked(bool toggled)
{

	if (toggled == true)
	{
		ui.Camera_Switch->setChecked(toggled);
		connect(realsense, SIGNAL(sendFrame(cv::Mat)), head, SLOT(receiveFrame(cv::Mat)), Qt::DirectConnection);

	}
	else if (toggled == false)
	{
		disconnect(realsense, SIGNAL(sendFrame(cv::Mat)), head, SLOT(receiveFrame(cv::Mat)));
	}

}

void GUI::on_Pose_Switch_clicked(bool toggled)
{
	if (toggled == true)
	{
		ui.Camera_Switch->setChecked(toggled);
		connect(realsense, SIGNAL(sendFrame(cv::Mat)), pose, SLOT(receiveFrame(cv::Mat)), Qt::DirectConnection);
	}
	else if (toggled == false)
	{
		disconnect(realsense, SIGNAL(sendFrame(cv::Mat)), pose, SLOT(receiveFrame(cv::Mat)));
	}
	/*
	ui.Pose_Switch->hide();

	QLabel* loading_label = new QLabel;
	QMovie* loading_movie = new QMovie(":/Image/Resources/ajax-loader.gif");


	loading_label->setMovie(loading_movie);
	loading_label->setGeometry(ui.Pose_Switch->rect());
	loading_label->setMinimumSize(QSize(640, 480));
	loading_label->setMaximumSize(QSize(640, 480));
	loading_label->setSizeIncrement(QSize(0, 0));
	loading_label->setBaseSize(QSize(0, 0));
	//loading_label->show();
	//loading_movie->start();

	qDebug("Pose starting...");

	qDebug("Pose started...");

	ui.Pose_Switch->show();
	//loading_movie->stop();

	emit Pose_toggled(toggled);


	delete loading_label;
	delete loading_movie;*/

//}


ImageItem::ImageItem(QQuickItem* parent) : QQuickPaintedItem(parent)
{

	this->current_image = QImage(640, 480, QImage::Format_RGB888);
}

void ImageItem::paint(QPainter* painter)
{
	QRectF bounding_rect = boundingRect();
	QImage scaled = this->current_image.scaledToHeight(bounding_rect.height());
	QPointF center = bounding_rect.center() - scaled.rect().center();

	if (center.x() < 0)
		center.setX(0);
	if (center.y() < 0)
		center.setY(0);
	painter->drawImage(center, scaled);

}

QImage ImageItem::image() const
{
	return this->current_image;
}

void ImageItem::setImage(const QImage image)
{
	this->current_image = image.rgbSwapped();
	qDebug() << GUI_MSG << "update";
	update();
	
}
FboInSGRenderer::FboInSGRenderer()
{
	
}
void FboInSGRenderer::get_posePointer(Pose* pointer)
{
	renderer->logo.pose = pointer;
}
void FboInSGRenderer::get_robotPointer(RobotArm* pointer)
{
	renderer->logo.robot = pointer;
}

QQuickFramebufferObject::Renderer* FboInSGRenderer::createRenderer() const
{
	renderer = new FboRenderer();

	return renderer;
}

FboInSGRenderer::~FboInSGRenderer()
{

}


QList<QThread*> ThreadRenderer::threads;

/*
 * The render thread shares a context with the scene graph and will
 * render into two separate FBOs, one to use for display and one
 * to use for rendering
 */


RenderThread::RenderThread(const QSize& size)
	: surface(nullptr)
	, context(nullptr)
	, m_renderFbo(nullptr)
	, m_displayFbo(nullptr)
	, m_logoRenderer(nullptr)
	, m_size(size)
{
	ThreadRenderer::threads << this;
	m_logoRenderer = new Opengl();
}



ThreadRenderer::ThreadRenderer()
	: m_renderThread(nullptr)
{
	setFlag(ItemHasContents, true);
	m_renderThread = new RenderThread(QSize(512, 512));
}


void ThreadRenderer::ready()
{
	m_renderThread->surface = new QOffscreenSurface();
	m_renderThread->surface->setFormat(m_renderThread->context->format());
	m_renderThread->surface->create();

	m_renderThread->moveToThread(m_renderThread);

	connect(this, SIGNAL(shut()), m_renderThread, SLOT(shutDown()), Qt::QueuedConnection);
	m_renderThread->start();
	m_renderThread->setPriority(QThread::LowPriority);
	update();
}

QSGNode* ThreadRenderer::updatePaintNode(QSGNode* oldNode, UpdatePaintNodeData*)
{
	TextureNode* node = static_cast<TextureNode*>(oldNode);

	if (!m_renderThread->context) {
		QOpenGLContext* current = window()->openglContext();
		// Some GL implementations requres that the currently bound context is
		// made non-current before we set up sharing, so we doneCurrent here
		// and makeCurrent down below while setting up our own context.
		current->doneCurrent();

		m_renderThread->context = new QOpenGLContext();
		m_renderThread->context->setFormat(current->format());
		m_renderThread->context->setShareContext(current);
		m_renderThread->context->create();
		m_renderThread->context->moveToThread(m_renderThread);
		current->makeCurrent(window());

		QMetaObject::invokeMethod(this, "ready");
		return nullptr;
	}

	if (!node) {
		node = new TextureNode(window());

		/* Set up connections to get the production of FBO textures in sync with vsync on the
		 * rendering thread.
		 *
		 * When a new texture is ready on the rendering thread, we use a direct connection to
		 * the texture node to let it know a new texture can be used. The node will then
		 * emit pendingNewTexture which we bind to QQuickWindow::update to schedule a redraw.
		 *
		 * When the scene graph starts rendering the next frame, the prepareNode() function
		 * is used to update the node with the new texture. Once it completes, it emits
		 * textureInUse() which we connect to the FBO rendering thread's renderNext() to have
		 * it start producing content into its current "back buffer".
		 *
		 * This FBO rendering pipeline is throttled by vsync on the scene graph rendering thread.
		 */
		connect(m_renderThread, &RenderThread::textureReady, node, &TextureNode::newTexture, Qt::DirectConnection);
		connect(node, &TextureNode::pendingNewTexture, window(), &QQuickWindow::update, Qt::QueuedConnection);
		connect(window(), &QQuickWindow::beforeRendering, node, &TextureNode::prepareNode, Qt::DirectConnection);
		connect(node, &TextureNode::textureInUse, m_renderThread, &RenderThread::renderNext, Qt::QueuedConnection);

		// Get the production of FBO textures started..
		QMetaObject::invokeMethod(m_renderThread, "renderNext", Qt::QueuedConnection);
	}

	node->setRect(boundingRect());

	return node;
}
